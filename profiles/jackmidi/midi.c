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

#define JACK_RINGBUFFER_SIZE 1024 // Quantity of bytes in each ble<>jack buffer
#define MAX_DEVICES 32 // Maximum quantity of concurrent BLE MIDI devices
#define MIDI_UUID "03B80E5A-EDE8-4B33-A751-6CE34EC4C700"
#define MIDI_IO_UUID "7772E5DB-3868-4112-A1A9-F2669D106BF3"


struct midi_read_parser {
	uint8_t rstatus;                 // running status byte
	uint8_t rlen;                    // quantity of bytes in the running status message
	int64_t rtime;                   // last reader's real time
	int16_t timestamp;               // last MIDI-BLE timestamp
	int8_t timestamp_low;            // MIDI-BLE timestampLow from the current packet
	int8_t timestamp_high;           // MIDI-BLE timestampHigh from the current packet
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
};

struct midi_event_t {
	uint8_t len;
	uint8_t cmd;
	uint8_t val1;
	uint8_t val2;
};

static struct midi * midis[MAX_DEVICES]; // Static array of pointers to midi structures
static jack_client_t * jack_client = NULL;
static bool jack_mutex = false;

static bool midi_write_cb(struct io *io, void *user_data)
{
	// Called by main loop when fd is ready
	// Handle MIDI messages received from other jack clients - send to bluetooth device
	//!@todo Use timetamps - currently always set to zero
	//!@todo Support sysex - currently silently ignored
	
	struct midi *midi = user_data;
	if (!midi || !midi->playback_buffer)
		return false;
	int err;
	uint64_t val;
	uint8_t ev[4];
	uint8_t count, ev_len, cmd, rstatus = 0;
	while ((err = read(midi->fd, &val, sizeof(uint64_t))) > 0) {
		// jack process thread has signalled there is data available in this BLE MIDI ringbuffer
		size_t len = jack_ringbuffer_read_space(midi->playback_buffer);
		char buffer[len + 1];
		size_t pos = 0;
		buffer[pos++] = 0x80; // Header / timestamp high
		for (int i = 0; i < val; ++i) {
			jack_ringbuffer_read(midi->playback_buffer, &ev_len, 1);
			jack_ringbuffer_read(midi->playback_buffer, &cmd, 1);
			if (cmd != rstatus) {
				buffer[pos++] = 0x80; // Timestamp low
				buffer[pos++] = cmd;
				if (cmd < 0x80 || cmd > 0xEF)
					rstatus = 0;
				else
					rstatus = cmd;
			}
			pos += jack_ringbuffer_read(midi->playback_buffer, buffer + pos, ev_len - 1);
		}
		/*
		printf("midi_write_cb read %lu MIDI message. %d bytes buffered: ", val, pos);
		for(int i = 0; i < pos; ++i)
			printf("%02X ", buffer[i]);
		printf("\n");
		*/

		bt_gatt_client_write_without_response(midi->client,
							midi->midi_io_handle,
							false,
							buffer,
							pos);
	}
	return true;
}

static void midi_io_notify_cb(uint16_t value_handle, const uint8_t *data,
                             uint16_t length, void *user_data)
{
	/* Handle message received from BLE device - send to jack port
		First byte is always a header including the most significant 6 bits of the BLE timestamp
		MIDI Command bytes are preceded with BLE timestamp least significant 7 bits
		Sysex continuation does not have a preceding timestamp
		Running status is only supported within a single BLE packet
	*/
	struct midi *midi = user_data;
	if (!midi || !midi->capture_buffer)
		return;
	struct midi_read_parser *parser = &(midi->midi_in);

	if (length < 3) {
		warn("MIDI I/O: Wrong packet format: length is %u bytes but it should "
		     "be at least 3 bytes", length);
		return;
	}

	parser->timestamp_high = ((data[0]) & 0b111111) << 7;
	uint16_t offset = 1;
	struct midi_event_t ev;
	//!@todo Reset running status

	while (offset < length) {
		if (data[offset] & 0x80) {
			// Timestamp
			parser->timestamp_low = data[offset] & 0x7F;
			//!@todo Do something with timestamp
			if (++offset >= length) {
				error("Timestamp without MIDI command");
				break;
			}
			if (data[offset] & 0x80) {
				// MIDI Command
			} else {
				// Sysex continuation
			}
		}

		switch (data[offset]) {
			case 0xF8 ... 0xFF:
				// Realtime messages
				ev.len = 1;
				ev.cmd = data[offset++];
				break;
			case 0xF0: // SysEx Start - pass all data until Sysex End
			case 0xF7: /* SysEx End */
				parser->rstatus = data[offset];
				parser->rlen = 1;
				ev.len = 1;
				ev.cmd = data[offset++];
				break;
			case 0xF1: // Timecode quarter frame
			case 0xF3: // Song select
				// System Common 2 byte messages
				ev.len = 2;
				ev.cmd = data[offset++];
				break;
			case 0xF2: // Song position pointer
				// System Common 3 byte messages
				ev.len = 3;
				ev.cmd = data[offset++];
				break;
			case 0xC0 ... 0xDF:
				// Channel messages with 1 byte parameter
				parser->rstatus = data[offset];
				parser->rlen = 2;
				ev.len = 2;
				ev.cmd = data[offset++];
				break;
			case 0x80 ... 0xBF:
			case 0xE0 ... 0xEF:
				// Channel messages with 2 byte parameters
				parser->rstatus = data[offset];
				parser->rlen = 3;
				ev.len = 3;
				ev.cmd = data[offset++];
				break;
			default:
				ev.len = parser->rlen;
				if (parser->rstatus == 0xF0)
					ev.cmd = data[offset++];
				else
					ev.cmd = parser->rstatus;
		}
		memcpy(&(ev.val1), data + offset, ev.len - 1);
		jack_ringbuffer_write(midi->capture_buffer, &(ev.len), ev.len + 1);
		offset += ev.len - 1;
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

	struct midi_event_t ev;
	jack_midi_event_t jev;
	for (int i = 0; i < MAX_DEVICES; ++i) {
		// Iterate each BLE device
		if (midis[i] == NULL)
			continue;
		struct midi *midi = midis[i];

		// Process messages from BLE device, sending to jack
		void* capture_buffer = jack_port_get_buffer(midi->capture_port, nFrames);
		jack_midi_clear_buffer(capture_buffer);
		while (true) {
			if (jack_ringbuffer_read(midi->capture_buffer, &(ev.len), 1) != 1)
				break; // No more data in ring buffer
			if (jack_ringbuffer_read(midi->capture_buffer, &(ev.cmd), ev.len) != ev.len)
				break; // MIDI data seems to be missing!
			if(jack_midi_event_write(capture_buffer, 0, &ev.cmd, ev.len)) {
				error("Insufficient space in jack MIDI buffer");
				break;
			}
			//!@todo Peek the data from ringbuffer then remove if able to write to jack
		}

		// Process messages from jack, sending to BLE device
		void* playback_buffer = jack_port_get_buffer(midi->playback_port, nFrames);
		uint64_t count = jack_midi_get_event_count(playback_buffer);
		for (jack_nframes_t i = 0; i < count; ++i) {
			if(jack_midi_event_get(&jev, playback_buffer, i))
				continue;
			if (jev.size < 4) {
				uint8_t len = jev.size;
				jack_ringbuffer_write(midi->playback_buffer, &len, 1);
				jack_ringbuffer_write(midi->playback_buffer, jev.buffer, jev.size);
			}
		}
		if (count)
			write(midi->fd, &count, sizeof(uint64_t));
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
