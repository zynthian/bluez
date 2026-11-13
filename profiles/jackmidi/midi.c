// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *  jack-midi - BLE MIDI jack interface
 *
 *  Copyright (C) 2015,2016 Felipe F. Tonello <eu@felipetonello.com>
 *  Copyright (C) 2016 ROLI Ltd.
 *  Copyright (C) 2024 riban ltd <info@riban.co.uk>
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
//#include <alsa/asoundlib.h>
#include <jack/jack.h> //provides JACK interface
#include <jack/midiport.h> //provides JACK MIDI interface
#include <jack/ringbuffer.h> // provides JACK ring buffer
#include <unistd.h> // provides file discriptor operations, e.g. close, read, etc.
#include <sys/eventfd.h> // provides event file descriptor eventfd

#include "lib/bluetooth.h"
#include "lib/sdp.h"
#include "lib/uuid.h"

#include "src/plugin.h"
#include "src/adapter.h"
#include "src/device.h"
#include "src/profile.h"
#include "src/service.h"
#include "src/shared/util.h"
#include "src/shared/att.h"
#include "src/shared/queue.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-client.h"
#include "src/shared/io.h"
#include "src/log.h"
#include "attrib/att.h"

//#include "libmidi.h"

#define JACK_RINGBUFFER_SIZE 4096 // Quantity of bytes in each ble<>jack buffer
#define MIDI_MAX_EVENT_SIZE 1024 // Largest MIDI message (i.e. SysEx) passed between BLE and jack
#define BLE_MIDI_MAX_PACKET 512 // Largest BLE-MIDI packet built (ATT MTU - 3, capped)
#define BLE_MIDI_DEFAULT_PACKET 20 // BLE-MIDI packet size for the default ATT MTU (23 - 3)
#define MAX_DEVICES 32 // Maximum quantity of concurrent BLE MIDI devices
#define MIDI_UUID "03B80E5A-EDE8-4B33-A751-6CE34EC4C700"
#define MIDI_IO_UUID "7772E5DB-3868-4112-A1A9-F2669D106BF3"

/*
 * Each ble<>jack ringbuffer holds whole MIDI messages, each stored as a
 * uint16_t length followed by the message bytes.
 */

struct midi_read_parser {
	uint8_t rstatus;                 // running status byte (0 if none)
	uint8_t msg[3];                  // short message being assembled
	uint8_t msg_pos;                 // quantity of bytes in msg
	uint8_t msg_len;                 // expected quantity of bytes in msg
	bool sysex;                      // true between F0 and F7
	bool sysex_overflow;             // current SysEx is too long - drop it
	uint16_t sysex_len;              // quantity of bytes in sysex_buf
	uint8_t sysex_buf[MIDI_MAX_EVENT_SIZE]; // SysEx being assembled across BLE packets
};

struct ble_midi_packet {
	uint8_t buf[BLE_MIDI_MAX_PACKET]; // buf[0] is the header byte
	uint16_t len;                    // quantity of bytes in buf
	uint16_t max;                    // maximum packet size for this connection
	uint8_t ts;                      // timestamp byte preceding each status byte
};

struct midi {
	struct btd_device *dev;
	struct gatt_db *db;
	struct bt_gatt_client *client;
	unsigned int io_cb_id;
	struct io *io;
	uint16_t midi_io_handle;
	int fd;

	/* JACK ports */
	jack_port_t * capture_port; // Jack MIDI capture port
	jack_port_t * playback_port;
	jack_ringbuffer_t * capture_buffer;
	jack_ringbuffer_t * playback_buffer;

	/* MIDI parser*/
	struct midi_read_parser midi_in;

	/* Quantity of jack messages dropped since last reported (jack thread increments) */
	unsigned int playback_dropped;
};

static struct midi * midis[MAX_DEVICES]; // Static array of pointers to midi structures
static jack_client_t * jack_client = NULL;
static bool jack_mutex = false;

/* Quantity of bytes in a MIDI message including its status byte (0 for SysEx) */
static uint8_t midi_msg_len(uint8_t status)
{
	switch (status) {
		case 0x80 ... 0xBF:
		case 0xE0 ... 0xEF:
		case 0xF2: // Song position pointer
			return 3;
		case 0xC0 ... 0xDF:
		case 0xF1: // Timecode quarter frame
		case 0xF3: // Song select
			return 2;
		case 0xF0:
			return 0;
		default:
			return 1; // Tune request, undefined, stray EOX, realtime
	}
}

/* Queue a whole MIDI message in a ble<>jack ringbuffer. Returns false (and
   queues nothing) if it does not fit. */
static bool rb_queue_msg(jack_ringbuffer_t *rb, const uint8_t *data, uint16_t len)
{
	if (!rb || !len || jack_ringbuffer_write_space(rb) < sizeof(len) + len)
		return false;
	jack_ringbuffer_write(rb, (const char *)&len, sizeof(len));
	jack_ringbuffer_write(rb, (const char *)data, len);
	return true;
}

/* Length of the next whole message in a ble<>jack ringbuffer, 0 if none */
static uint16_t rb_next_msg_len(jack_ringbuffer_t *rb)
{
	uint16_t len;

	if (!rb || jack_ringbuffer_peek(rb, (char *)&len, sizeof(len)) != sizeof(len))
		return 0;
	if (jack_ringbuffer_read_space(rb) < sizeof(len) + len)
		return 0; // Writer has not finished queuing it
	return len;
}

/* --- jack -> BLE --- */

static void ble_packet_flush(struct midi *midi, struct ble_midi_packet *pkt)
{
	if (pkt->len > 1 && !bt_gatt_client_write_without_response(midi->client,
						midi->midi_io_handle, false, pkt->buf, pkt->len))
		error("MIDI I/O: failed to write %u byte BLE-MIDI packet", pkt->len);
	pkt->len = 1; // Keep the header for the next packet
}

/* Start a new packet unless n more bytes fit in this one */
static void ble_packet_reserve(struct midi *midi, struct ble_midi_packet *pkt, uint16_t n)
{
	if (pkt->len + n > pkt->max)
		ble_packet_flush(midi, pkt);
}

static void ble_packet_add_short(struct midi *midi, struct ble_midi_packet *pkt,
                                 const uint8_t *msg, uint8_t len)
{
	// Status byte is always sent (no running status) so each message is self-contained
	ble_packet_reserve(midi, pkt, len + 1);
	pkt->buf[pkt->len++] = pkt->ts;
	memcpy(pkt->buf + pkt->len, msg, len);
	pkt->len += len;
}

/* SysEx per BLE-MIDI 1.0: timestamp, F0, data... split over as many packets as
   required, each continuation packet being header + data with no timestamp,
   then timestamp, F7. */
static void ble_packet_add_sysex(struct midi *midi, struct ble_midi_packet *pkt,
                                 const uint8_t *msg, uint16_t len)
{
	bool eox = len > 1 && msg[len - 1] == 0xF7;
	uint16_t end = eox ? len - 1 : len;

	ble_packet_reserve(midi, pkt, 3); // Timestamp, F0, first data byte
	pkt->buf[pkt->len++] = pkt->ts;
	pkt->buf[pkt->len++] = 0xF0;
	for (uint16_t i = 1; i < end; ++i) {
		if (pkt->len >= pkt->max)
			ble_packet_flush(midi, pkt);
		pkt->buf[pkt->len++] = msg[i];
	}
	if (eox) {
		ble_packet_reserve(midi, pkt, 2);
		pkt->buf[pkt->len++] = pkt->ts;
		pkt->buf[pkt->len++] = 0xF7;
	}
}

static bool midi_write_cb(struct io *io, void *user_data)
{
	// Called by main loop when the jack process thread signals the eventfd
	// Send MIDI messages received from other jack clients to the bluetooth device

	struct midi *midi = user_data;
	if (!midi || !midi->playback_buffer)
		return false;

	uint64_t val;
	while (read(midi->fd, &val, sizeof(val)) > 0)
		; // Clear the eventfd - the ringbuffer is drained regardless of the count

	unsigned int dropped = __atomic_exchange_n(&midi->playback_dropped, 0, __ATOMIC_RELAXED);
	if (dropped)
		warn("MIDI I/O: dropped %u jack MIDI messages (too long or buffer full)", dropped);

	struct ble_midi_packet pkt;
	uint16_t mtu = bt_gatt_client_get_mtu(midi->client);
	pkt.max = mtu > 23 ? MIN(mtu - 3, BLE_MIDI_MAX_PACKET) : BLE_MIDI_DEFAULT_PACKET;
	uint16_t ts = (g_get_monotonic_time() / 1000) & 0x1FFF; // 13-bit milliseconds
	pkt.buf[0] = 0x80 | (ts >> 7);
	pkt.ts = 0x80 | (ts & 0x7F);
	pkt.len = 1;

	uint8_t msg[MIDI_MAX_EVENT_SIZE];
	uint16_t len;
	while ((len = rb_next_msg_len(midi->playback_buffer))) {
		jack_ringbuffer_read_advance(midi->playback_buffer, sizeof(len));
		jack_ringbuffer_read(midi->playback_buffer, (char *)msg, len);
		if (msg[0] == 0xF0)
			ble_packet_add_sysex(midi, &pkt, msg, len);
		else if ((msg[0] & 0x80) && len >= midi_msg_len(msg[0]))
			ble_packet_add_short(midi, &pkt, msg, midi_msg_len(msg[0]));
		else
			DBG("MIDI I/O: ignoring malformed %u byte jack MIDI message", len);
	}
	ble_packet_flush(midi, &pkt);
	return true;
}

/* --- BLE -> jack --- */

static void midi_in_emit(struct midi *midi, const uint8_t *data, uint16_t len)
{
	if (!rb_queue_msg(midi->capture_buffer, data, len))
		warn("MIDI I/O: jack capture buffer full - dropped %u byte message", len);
}

static void midi_in_status(struct midi *midi, uint8_t status)
{
	struct midi_read_parser *p = &midi->midi_in;

	if (status >= 0xF8) {
		// Realtime - may appear anywhere, including inside SysEx
		midi_in_emit(midi, &status, 1);
		return;
	}

	if (p->sysex) {
		p->sysex = false;
		if (status == 0xF7) {
			if (p->sysex_overflow) {
				warn("MIDI I/O: dropped SysEx longer than %d bytes", MIDI_MAX_EVENT_SIZE);
			} else {
				p->sysex_buf[p->sysex_len++] = 0xF7;
				midi_in_emit(midi, p->sysex_buf, p->sysex_len);
			}
			return;
		}
		warn("MIDI I/O: dropped SysEx interrupted by status 0x%02X", status);
		// Handle the interrupting status byte below
	}

	p->msg_pos = 0;
	if (status == 0xF0) {
		p->sysex = true;
		p->sysex_overflow = false;
		p->sysex_buf[0] = 0xF0;
		p->sysex_len = 1;
		p->rstatus = 0;
		return;
	}
	if (status == 0xF7)
		return; // EOX without SysEx

	p->rstatus = status < 0xF0 ? status : 0; // System Common cancels running status
	p->msg[0] = status;
	p->msg_len = midi_msg_len(status);
	p->msg_pos = 1;
	if (p->msg_len == 1) {
		midi_in_emit(midi, p->msg, 1);
		p->msg_pos = 0;
	}
}

static void midi_in_data(struct midi *midi, uint8_t data)
{
	struct midi_read_parser *p = &midi->midi_in;

	if (p->sysex) {
		// Leave room for F7
		if (p->sysex_len < MIDI_MAX_EVENT_SIZE - 1)
			p->sysex_buf[p->sysex_len++] = data;
		else
			p->sysex_overflow = true;
		return;
	}
	if (p->msg_pos == 0) {
		if (!p->rstatus)
			return; // Data byte without status
		p->msg[0] = p->rstatus;
		p->msg_len = midi_msg_len(p->rstatus);
		p->msg_pos = 1;
	}
	p->msg[p->msg_pos++] = data;
	if (p->msg_pos >= p->msg_len) {
		midi_in_emit(midi, p->msg, p->msg_len);
		p->msg_pos = 0;
	}
}

static void midi_io_notify_cb(uint16_t value_handle, const uint8_t *data,
                             uint16_t length, void *user_data)
{
	/* Handle a BLE-MIDI packet received from the BLE device - send to jack port
		First byte is a header (bit 7 set) holding the timestamp high bits
		Each status byte is preceded by a timestamp byte (bit 7 set)
		Running status data may follow, with or without its own timestamp
		SysEx may span packets: continuation packets carry data straight
		after the header, and F7 is preceded by its own timestamp
		Timestamps are ignored - messages are sent to jack as they arrive
	*/
	struct midi *midi = user_data;
	if (!midi || !midi->capture_buffer)
		return;
	struct midi_read_parser *p = &midi->midi_in;

	if (length < 2 || !(data[0] & 0x80)) {
		warn("MIDI I/O: invalid BLE-MIDI packet (%u bytes)", length);
		return;
	}

	p->msg_pos = 0; // A short message may not span packets

	for (uint16_t i = 1; i < length; ++i) {
		uint8_t b = data[i];
		if (!(b & 0x80)) {
			midi_in_data(midi, b); // SysEx data or running status data
			continue;
		}
		// Timestamp, followed by a status byte or running status data
		if (i + 1 >= length) {
			if (p->sysex && b == 0xF7)
				midi_in_status(midi, b); // Tolerate EOX sent without timestamp
			else
				warn("MIDI I/O: timestamp without MIDI message");
			break;
		}
		uint8_t next = data[++i];
		if (next & 0x80) {
			midi_in_status(midi, next);
		} else {
			if (p->sysex && b == 0xF7)
				midi_in_status(midi, b); // Tolerate EOX sent without timestamp
			else if (!p->sysex)
				p->msg_pos = 0; // Timestamp starts a new running status message
			midi_in_data(midi, next);
		}
	}
}

static void midi_io_register_cb(uint16_t att_ecode, void *user_data)
{
	if (att_ecode != 0) {
		error("MIDI I/O: notifications not enabled %s",
		      att_ecode2str(att_ecode));
		return;
	}

	DBG("MIDI I/O: notification enabled");
}

static void midi_io_initial_read_cb(bool success, uint8_t att_ecode,
                                    const uint8_t *value, uint16_t length,
                                    void *user_data)
{
	struct midi *midi = user_data;

	if (!success) {
		error("MIDI I/O: Failed to read initial request");
		return;
	}

	/* request notify */
	midi->io_cb_id =
		bt_gatt_client_register_notify(midi->client,
		                               midi->midi_io_handle,
		                               midi_io_register_cb,
		                               midi_io_notify_cb,
		                               midi,
		                               NULL);
}

static void handle_midi_io(struct midi *midi, uint16_t value_handle)
{
	DBG("MIDI I/O handle: 0x%04x", value_handle);

	midi->midi_io_handle = value_handle;

	/*
	 * The BLE-MIDI 1.0 spec specifies that The Central shall attempt to
	 * read the MIDI I/O characteristic of the Peripheral right after
	 * establishing a connection with the accessory.
	 */
	if (!bt_gatt_client_read_value(midi->client,
	                               value_handle,
	                               midi_io_initial_read_cb,
	                               midi,
	                               NULL))
		DBG("MIDI I/O: Failed to send request to read initial value");
}

static void handle_characteristic(struct gatt_db_attribute *attr,
                                  void *user_data)
{
	struct midi *midi = user_data;
	uint16_t value_handle;
	bt_uuid_t uuid, midi_io_uuid;

	if (!gatt_db_attribute_get_char_data(attr, NULL, &value_handle, NULL,
	                                     NULL, &uuid)) {
		error("Failed to obtain characteristic data");
		return;
	}

	bt_string_to_uuid(&midi_io_uuid, MIDI_IO_UUID);

	if (bt_uuid_cmp(&midi_io_uuid, &uuid) == 0)
		handle_midi_io(midi, value_handle);
	else {
		char uuid_str[MAX_LEN_UUID_STR];

		bt_uuid_to_string(&uuid, uuid_str, sizeof(uuid_str));
		DBG("Unsupported characteristic: %s", uuid_str);
	}
}

static void foreach_midi_service(struct gatt_db_attribute *attr,
                                 void *user_data)
{
	struct midi *midi = user_data;

	gatt_db_service_foreach_char(attr, handle_characteristic, midi);
}

static int midi_device_probe(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct midi *midi;

	// Debug only
	char addr[18];
	ba2str(device_get_address(device), addr);
	DBG("MIDI GATT Driver profile probe (%s)", addr);

	/* Ignore, if we were probed for this device already */
	midi = btd_service_get_user_data(service);
	if (midi) {
		error("Profile probed twice for the same device!");
		return -EADDRINUSE;
	}

	midi = g_new0(struct midi, 1);
	if (!midi)
		return -ENOMEM;

	midi->dev = btd_device_ref(device);

	btd_service_set_user_data(service, midi);

	return 0;
}

static void midi_device_remove(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct midi *midi;

	// Debug only
	char addr[18];
	ba2str(device_get_address(device), addr);
	DBG("MIDI GATT Driver profile remove (%s)", addr);

	midi = btd_service_get_user_data(service);
	if (!midi) {
		error("MIDI Service not handled by profile");
		return;
	}

	for (int i =0; i < MAX_DEVICES; ++i) {
		if (midis[i] != midi)
			continue;
		while(jack_mutex)
			g_usleep(200);
		midis[i] = NULL;
		jack_port_unregister(jack_client, midi->capture_port);
		jack_port_unregister(jack_client, midi->playback_port);
		io_destroy(midi->io);
		close(midi->fd);
		break;
	}

	btd_device_unref(midi->dev);
	g_free(midi);
}

static int midi_accept(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct gatt_db *db = btd_device_get_gatt_db(device);
	struct bt_gatt_client *client = btd_device_get_gatt_client(device);
	bt_uuid_t midi_uuid;
	struct midi *midi;
	char addr[18];
	char device_name[jack_port_name_size() - 26]; // -26 "BLE:XX:XX:XX:XX:XX:XX/ OUT"
	int err;

	ba2str(device_get_address(device), addr);
	DBG("MIDI GATT Driver profile accept (%s)", addr);

	midi = btd_service_get_user_data(service);
	if (!midi) {
		error("MIDI Service not handled by profile");
		return -ENODEV;
	}

	size_t ptr = 0;
	char id_str [13]; // addr without colons
	for (int i = 0; i < strlen(addr); ++i) {
		if (addr[i] != ':')
			id_str[ptr++] = addr[i];
	}
	id_str[ptr] = '\0';

	// Get the device name or addr if no name for this device known
	if (device_name_known(device))
		device_get_name(device, device_name, sizeof(device_name));
	else
		sprintf(device_name, "%s", id_str);

	//!@todo Start jack client if it is not started

	// Create jack ports
	if (jack_client) {
		char port_name[jack_port_name_size()];
		sprintf(port_name, "%s %s out", device_name, id_str);
	    if(!(midi->playback_port = jack_port_register(jack_client, port_name, JACK_DEFAULT_MIDI_TYPE, JackPortIsInput | JackPortIsPhysical, 0)))
    	    error("Cannot register jack playback port for %s\n", device_name);
		else {
			sprintf(port_name, "BLE:%s/%s OUT", addr, device_name); // reuse port_name for alias[0]
			jack_port_set_alias(midi->playback_port, port_name);
			jack_port_set_alias(midi->playback_port, device_name);
			midi->playback_buffer = jack_ringbuffer_create(JACK_RINGBUFFER_SIZE);
		}
		sprintf(port_name, "%s %s in", device_name, id_str);
	    if(!(midi->capture_port = jack_port_register(jack_client, port_name, JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput | JackPortIsPhysical, 0)))
    	    error("Cannot register jack capture port for %s\n", device_name);
		else {
			sprintf(port_name, "BLE:%s/%s IN", addr, device_name); // reuse port_name for alias[0]
			jack_port_set_alias(midi->capture_port, port_name);
			jack_port_set_alias(midi->capture_port, device_name);
			midi->capture_buffer = jack_ringbuffer_create(JACK_RINGBUFFER_SIZE);
		}
		printf("Created jack MIDI ports for %s\n", device_name);

		// Create event file descriptor used to signal rx data
		midi->fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
		midi->io = io_new(midi->fd);
		if (!midi->io)
			error("Could not allocate i/o event loop");
	}

	io_set_read_handler(midi->io, midi_write_cb, midi, NULL);

	memset(&midi->midi_in, 0, sizeof(midi->midi_in)); // Forget any previous connection's state
	midi->playback_dropped = 0;

	midi->db = gatt_db_ref(db);
	midi->client = bt_gatt_client_ref(client);

	for (int i = 0; i < MAX_DEVICES; ++i) {
		if (midis[0] != NULL)
			continue;
		midis[i] = midi;
		break;
	}

	bt_string_to_uuid(&midi_uuid, MIDI_UUID);
	gatt_db_foreach_service(db, &midi_uuid, foreach_midi_service, midi);

	btd_service_connecting_complete(service, 0);

	return 0;

	btd_service_connecting_complete(service, err);

	return err;
}

static int midi_disconnect(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct midi *midi;

	// Debug only
	char addr[18];
	ba2str(device_get_address(device), addr);
	DBG("MIDI GATT Driver profile disconnect (%s)", addr);

	midi = btd_service_get_user_data(service);
	if (!midi) {
		error("MIDI Service not handled by profile");
		return -ENODEV;
	}

	bool more_jack_ports = false;
	for (int i = 0; i < MAX_DEVICES; ++i) {
		if (midis[i] != midi) {
			if (midis[i])
				more_jack_ports = true;
			continue;
		}
		while(jack_mutex)
			g_usleep(200);
		midis[i] = NULL;
		jack_port_unregister(jack_client, midi->playback_port);
		midi->playback_port = NULL;
		jack_port_unregister(jack_client, midi->capture_port);
		midi->capture_port = NULL;
		jack_ringbuffer_free(midi->playback_buffer);
		midi->playback_buffer = NULL;
		jack_ringbuffer_free(midi->capture_buffer);
		midi->capture_buffer = NULL;
		close(midi->fd);
		midi->fd = 0;
		printf("Removed jack MIDI ports for %s\n", addr);
		break;
	}
	//!@todo Close jack client if no more ports registered

	io_destroy(midi->io);

	/* Clean-up any old client/db */
	bt_gatt_client_unregister_notify(midi->client, midi->io_cb_id);
	bt_gatt_client_unref(midi->client);
	gatt_db_unref(midi->db);

	btd_service_disconnecting_complete(service, 0);
	return 0;
}

static struct btd_profile midi_profile = {
	.name = "MIDI GATT Driver",
	.remote_uuid = MIDI_UUID,
	.priority = BTD_PROFILE_PRIORITY_HIGH,
	.auto_connect = true,

	.device_probe = midi_device_probe,
	.device_remove = midi_device_remove,

	.accept = midi_accept,

	.disconnect = midi_disconnect,
};

static int jack_process(jack_nframes_t nFrames, void *pArgs) {
	jack_mutex = true;

	jack_midi_event_t jev;
	for (int i = 0; i < MAX_DEVICES; ++i) {
		// Iterate each BLE device
		if (midis[i] == NULL)
			continue;
		struct midi *midi = midis[i];

		// Process messages from BLE device, sending to jack
		if (midi->capture_port) {
			void* capture_buffer = jack_port_get_buffer(midi->capture_port, nFrames);
			jack_midi_clear_buffer(capture_buffer);
			bool empty = true;
			uint16_t len;
			while ((len = rb_next_msg_len(midi->capture_buffer))) {
				jack_midi_data_t *dst = jack_midi_event_reserve(capture_buffer, 0, len);
				if (!dst && !empty)
					break; // jack buffer full - send the rest next period
				jack_ringbuffer_read_advance(midi->capture_buffer, sizeof(len));
				if (dst)
					jack_ringbuffer_read(midi->capture_buffer, (char *)dst, len);
				else
					jack_ringbuffer_read_advance(midi->capture_buffer, len); // Never fits
				empty = false;
			}
		}

		// Process messages from jack, sending to BLE device
		if (midi->playback_port) {
			void* playback_buffer = jack_port_get_buffer(midi->playback_port, nFrames);
			uint64_t count = jack_midi_get_event_count(playback_buffer);
			for (uint32_t j = 0; j < count; ++j) {
				if (jack_midi_event_get(&jev, playback_buffer, j))
					continue;
				if (jev.size > MIDI_MAX_EVENT_SIZE ||
				    !rb_queue_msg(midi->playback_buffer, jev.buffer, jev.size))
					__atomic_add_fetch(&midi->playback_dropped, 1, __ATOMIC_RELAXED);
			}
			if (count)
				write(midi->fd, &count, sizeof(uint64_t));
		}
	}

	jack_mutex = false;
	return 0;
}

static int jackmidi_init(void)
{
	while(jack_mutex)
		g_usleep(400);
	for (int i = 0; i < MAX_DEVICES; ++i)
		midis[i] = NULL;
	//!@todo We could wait until a BLE MIDI device tries to connect before starting jack client
	char *sServerName = NULL;
	jack_status_t nStatus;
	if(!(jack_client = jack_client_open("BLE_MIDI", JackNoStartServer, &nStatus, sServerName)))
		error("Failed to start jack client 'BLE_MIDI' - error code: %d\n", nStatus);
	else {
		if(jack_set_process_callback(jack_client, jack_process, NULL))
			error("Failed to set jack process callback");
		else
			jack_activate(jack_client);
	}
	return btd_profile_register(&midi_profile);
}

static void jackmidi_exit(void)
{
	//!@todo We could stop jack client when last BLE MIDI device disconnects
	if (jack_client)
		jack_client_close(jack_client);
	jack_client = NULL;
	while(jack_mutex)
		g_usleep(400);
	for (int i = 0; i < 0; ++i) {
		if (midis[i] == NULL)
			continue;
		jack_ringbuffer_free(midis[i]->playback_buffer);
		jack_ringbuffer_free(midis[i]->capture_buffer);
		midis[i] = NULL;
	}
	btd_profile_unregister(&midi_profile);
}

BLUETOOTH_PLUGIN_DEFINE(jackmidi, VERSION, BLUETOOTH_PLUGIN_PRIORITY_HIGH,
                        jackmidi_init, jackmidi_exit);
