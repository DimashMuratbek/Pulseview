/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2021 Ashwin Nair <ashwin.nair@infineon.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <glib.h>
#include <glib/gstdio.h>
#include "protocol.h"

#include <stdint.h>
#include <stdio.h>

#include <libusb.h>

#define CMD_RESET 0xb3

#pragma pack(push, 1)



struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint16_t sampling_factor;
};

#pragma pack(pop)

#define USB_TIMEOUT 100


#define PREAMBLE 0xABCD    
#define HEADER_SIZE 16       // Up to start of Sample[0]
#define MAX_PACKET_SIZE 1024 
#define MAX_SAFE_SAMPLES 10  // Only up to 10 samples

// static uint16_t read_uint16_le(const uint8_t *buf) {
//     return (buf[1] << 8) | buf[0];
// }

static inline uint16_t read_uint16_be(const uint8_t *buf) {
    return (buf[0] << 8) | buf[1];
}


static struct parsed_packet parsed_pkt;

// Old checksum function
// static uint16_t calculate_checksum(const uint8_t *data, size_t length) {
//     uint16_t sum = 0;
//     for (size_t i = 0; i < length; i++) {
//         sum += data[i];
//     }
//     return sum & 0xFFFF;
// }

static uint16_t calculate_checksum(const uint8_t *data, size_t length)
{
    uint32_t sum = 0;

    /* Skip first 2 bytes (preamble 0xABCD), and add words in little-endian order */
    for (size_t i = 2; i + 1 < length; i += 2) {
        uint16_t word = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
        sum += word;
    }

    return (uint16_t)(sum & 0xFFFF);
}

int fx3driver_parse_next_packet(const uint8_t *data, size_t len, struct parsed_packet *pkt)
{



	//Display raw data
    for (size_t i=0;i<len;i++) {
		printf("%02X ", data[i]);
		if (i % 30 == 0) {
			printf("\n");
		
		}
	}

	// return 0;
	sr_err("Entered fx3driver_parse_next_packet (len=%zu)", len);
	//sr_err("Entered fx3driver_parse_next_packet (len=0x%zx)", len);



	if (!data || !pkt)
    return 0;

	memset(pkt, 0, sizeof(*pkt));

	
	
	size_t offset = 0;

	while (offset + 20 <= len) {
		if (read_uint16_be(&data[offset]) == 0xABCD) {
			sr_err("Dumping 16 words after preamble at offset %zu", offset);
			for (size_t i = 0; i < 16; i++) {
				uint16_t word = read_uint16_be(&data[offset + 2 + i * 2]);
				sr_err("Word[%zu] = 0x%04X", i, word);
			}

			uint16_t ch_field = read_uint16_be(&data[offset + 2]);
			uint8_t ch_type = ch_field >> 8;
			uint16_t length = read_uint16_be(&data[offset + 8]);
			uint16_t res1 = read_uint16_be(&data[offset + 10]);
			uint16_t res2 = read_uint16_be(&data[offset + 12]);
			uint16_t res3 = read_uint16_be(&data[offset + 14]);


			sr_err("Candidate offset %zu: ch=0x%04X type=0x%02X len=0x%04X res1=0x%04X res2=0x%04X res3=0x%04X",
				offset, ch_field, ch_type, length, res1, res2, res3);

			if ((ch_type == 0x00 || ch_type == 0xFF) &&
				(length >= 6 && length <= MAX_PACKET_SIZE) &&
				res1 == 0xF1F1 &&
				res2 == 0xF2F2 &&
				res3 == 0xF3F3) {
				sr_err("Valid preamble+header at offset %zu", offset);
				break;
			} else {
				sr_err("Skipping candidate at offset %zu due to invalid header", offset);
			}
		}
		offset++;
	}


	const uint8_t *pkt_data = &data[offset + 2];
    

	uint16_t channel_field = read_uint16_be(&pkt_data[0]);
	pkt->channel_type = (channel_field >> 8);
	sr_err("Channel type: 0x%04X", pkt->channel_type);

	pkt->channel_number = channel_field & 0xFF;

	size_t num_channels = pkt->channel_number; // you already did this
	pkt->num_channels = num_channels; 

	sr_err("Channel number: 0x%04X", pkt->channel_number);

	uint16_t ts_lo = read_uint16_be(&pkt_data[2]);
	sr_err("ts_lo: 0x%04X", ts_lo);

	uint16_t ts_hi = read_uint16_be(&pkt_data[4]);
	sr_err("ts_hi: 0x%04X", ts_hi);


	pkt->ts_lo = ts_lo;
	pkt->ts_hi = ts_hi;

	// uint16_t packet_length = read_uint16_be(&pkt_data[6]);
	// if (packet_length < 20 || len - offset < packet_length) {
	// 	sr_err("Invalid packet length: 0x%02X", packet_length);
	// 	return 2;
	// }

	// if (read_uint16_be(&pkt_data[8]) != 0xF1F1 ||
	// 	read_uint16_be(&pkt_data[10]) != 0xF2F2 ||
	// 	read_uint16_be(&pkt_data[12]) != 0xF3F3) {
	// 	sr_err("Reserved fields mismatch");
	// 	return 2;
	// }

	// Read LEN field (6 + sample_bytes)
  // corresponds to firmware's len_after

	uint16_t packet_length = read_uint16_be(&pkt_data[6]); 
	// Sanity check
	if (packet_length < 6) {
		sr_err("Invalid LEN (too small): %u", packet_length);
		return 2;
	}
	
	// Calculate true payload and packet sizes
	size_t sample_bytes = (size_t)packet_length - 6;         // just the sample section
	size_t total_packet_bytes = 16 + sample_bytes;       // header (16) + samples

	if (len < offset + total_packet_bytes) {
		sr_err("Truncated packet: need %zu, have %zu", offset + total_packet_bytes, len);
		return 0; // not enough data yet
	}

	sr_err("LEN=%u sample_bytes=%zu total_packet_bytes=%zu", packet_length, sample_bytes, total_packet_bytes);


	// Use sample_bytes for parsing
	size_t sample_data_len = sample_bytes;


	sr_err("Sample_data_length: %zu ",sample_data_len );

    //size_t sample_data_len = packet_length - 16;  // subtract header + checksum   use 18 if checksum enabled in packet and if not use 16
    //size_t num_samples = sample_data_len / 2; for digital
	
    // if (num_samples == 0 || num_samples > 16) {
        
	// 	sr_err("Invalid sample count: 0x%zx", num_samples);

    //     return 2;
    // }

    

    //pkt->digital_samples = g_malloc0(num_samples);
 

	if (pkt->channel_type == 0xFF){
		// Digital samples
		
		uint16_t channel_field = read_uint16_be(&pkt_data[0]);
		pkt->channel_type   = (channel_field >> 8);
		pkt->channel_number = channel_field & 0xFF;


		size_t dig_num_channels = pkt->num_channels;
		size_t num_samples = sample_data_len / 2; 
		pkt->num_samples = num_samples;
    	
    	pkt->digital_samples = g_malloc0(num_samples);
		
		if (!pkt->digital_samples) {
			sr_err("Memory allocation failed");
			return 2;
		}

		sr_err("Show me number of samples digital channels: %zu", dig_num_channels);

		sr_err("Show me number of samples/length: %zu", num_samples);
		//Get the bits from hex values for channels
		for (size_t i = 0; i < num_samples; i++) {

			uint16_t raw = read_uint16_be(&pkt_data[14 + i * 2]); // start getting samples  changed gferom 14 to 16 
			
			uint8_t raw_8 = raw & 0xFF; // take low 8 bits only

			if (dig_num_channels < 8) {
        		uint8_t mask = (1u << dig_num_channels) - 1u;
        		raw_8 &= mask;   // keep only that many bits
    		}

		// Optionally print each bit of the samples 
			for (size_t ch = 0; ch < dig_num_channels; ch++) {
				uint8_t bit = (raw_8 >> ch) & 1;
				sr_err("Sample[%zu] D%d: %s", i, ch, bit ? "HIGH" : "LOW");
			}

		// Save lower 8 bits as channel states
			pkt->digital_samples[i] = raw_8;
			sr_err("Raw sample[%zu] = 0x%04X", i, pkt->digital_samples[i]);



		}

		sr_err("Digital packet parsed successfully");

	
	}else if (pkt->channel_type == 0x00){
		
		size_t sample_data_offset = 14;              // first sample byte in pkt_data
		size_t num_channels = pkt->num_channels; //  12
		size_t sample_bytes = sample_data_len; 

		if (sample_bytes < num_channels) {
			sr_err("Analog payload too small: %zu bytes for %zu channels",
				sample_bytes, num_channels);
			return 2;
		}

		// Drop incomplete tail that can't form a full sample across all channels
		size_t remainder = sample_bytes % num_channels;
		if (remainder) {
			sr_err("Analog payload %zu not divisible by %zu channels, " "truncating last %zu bytes", sample_bytes, num_channels, remainder);
			sample_bytes -= remainder;
		}

		if (sample_bytes == 0) {
			sr_err("No complete analog samples after truncation.");
			return 2;
		}

		size_t num_samples_per_channel = sample_bytes / num_channels;
		size_t total_floats = num_samples_per_channel * num_channels;

		pkt->num_samples = num_samples_per_channel;
		pkt->analog_samples = g_malloc0(total_floats * sizeof(float));

		if (!pkt->analog_samples) {
			sr_err("Memory allocation failed");
			return 2;
		}

		sr_err("Dumping raw analog payload:");
		for (size_t i = 0; i < sample_bytes; i++) {
			uint8_t raw = pkt_data[sample_data_offset + i];
			float v = -((raw / 255.0f) * 30.0f);
			sr_err("RAW[%02zu] = 0x%02X -> %.3f V", i, raw, v);
		}

		sr_err("THE SAMPLE DATA LENGTH (aligned): %zu", sample_bytes);
		sr_err("Analog samples per channel: %zu", pkt->num_samples);
		
		sr_err("NUMBER OF CHANNELS : %zu", pkt->channel_number);
		

		memset(pkt->analog_samples, 0, num_samples_per_channel * num_channels * sizeof(float));
  
		for (size_t ch = 0; ch < num_channels; ch++) {
			for (size_t s = 0; s < num_samples_per_channel; s++) {
				size_t index = sample_data_offset + (ch * num_samples_per_channel) + s;

				if (index >= sample_data_offset + sample_bytes) {
					sr_err("Index %zu beyond analog payload (offset %zu, size %zu)", index, sample_data_offset, sample_bytes); // skip if beyond payload
					continue;
				}

				uint8_t raw = pkt_data[index];
				float voltage = -((raw / 255.0f) * 4.0f);
				//float voltage = (float)raw * 3.3f / 65535.0f;


				pkt->analog_samples[s * num_channels + ch] = voltage;
				sr_err("Sample[%zu] Channel[%zu]: %.3f V", s, ch, voltage);
			}
		}

    	sr_err("Analog packet parsed successfully");


	}

	// Compute checksum over the whole packet including preamble
	// const uint8_t *packet_start = &data[offset]; // start from preamble

	// uint16_t stored_checksum = read_uint16_be(&packet_start[packet_length - 2]);

	// uint16_t computed_checksum = 0;
	// for (size_t i = 0; i < packet_length - 2; i += 2) {
	// 	uint16_t word = read_uint16_be(&packet_start[i]);
	// 	computed_checksum = (computed_checksum + word);
	// 	sr_err("Checksum word[%zu]: 0x%04X, running total: 0x%04X", i / 2, word, computed_checksum);
	// }


	// sr_err("Computed checksum: 0x%04X", computed_checksum);

	// // Compare
	// if (computed_checksum != stored_checksum) {
	// 	sr_err("Checksum mismatch! Computed=0x%04X Stored=0x%04X", computed_checksum, stored_checksum);
	// 	return 2;
	// }

	// sr_err("Checksum OK");

    //return offset + packet_length;

	return offset + total_packet_bytes;
}





static void print_hex_debug_reversed(const uint8_t *buf, size_t len) {
    char line[128];

    for (ssize_t i = len - 1; i >= 0; i -= 16) {
        int n = snprintf(line, sizeof(line), "%04zx: ", (size_t)i);

        // j counts down from i to i - 15, but stops at 0
        for (ssize_t j = 0; j < 16 && (i - j) >= 0; j++) {
            n += snprintf(line + n, sizeof(line) - n, "%02X ", buf[i - j]);
        }

        sr_err("%s", line);
    }
}



static void print_hex_debug(const uint8_t *buf, size_t len) {
    char line[128];
    for (size_t i = 0; i < len; i += 16) {
        int n = snprintf(line, sizeof(line), "%04zx: ", i);
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            n += snprintf(line + n, sizeof(line) - n, "%02X ", buf[i + j]);
        }
        sr_err("%s", line);
    }
}




static int command_get_fw_version(libusb_device_handle *devhdl,
				  struct version_info *vi)
{
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_FW_VERSION, 0x0000, 0x0000,
		(unsigned char *)vi, sizeof(struct version_info), USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get version info: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_get_revid_version(struct sr_dev_inst *sdi, uint8_t *revid)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_device_handle *devhdl = usb->devhdl;
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, CMD_GET_REVID_VERSION, 0x0000, 0x0000,
		revid, 1, USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get REVID: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint64_t samplerate;
	struct cmd_start_acquisition cmd;
	int ret;

	devc = sdi->priv;
	usb = sdi->conn;
	samplerate = devc->cur_samplerate;

	/* Compute the sample rate. */
	if (devc->sample_wide && samplerate > MAX_16BIT_SAMPLE_RATE) {
		sr_err("Unable to sample at %" PRIu64 "Hz "
		       "when collecting 16-bit samples.", samplerate);
		return SR_ERR;
	}

	cmd.sampling_factor = (FX3_PIB_CLOCK)/(samplerate);
	
	sr_spew("cmd.sampling_factor = %d",cmd.sampling_factor);

	/* Send the control message. */
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, CMD_START, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send start command: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}else{
		sr_info("CMD_START vendor command sent successfully");
	}

	return SR_OK;
}

SR_PRIV int cypress_fx3_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct version_info vi;
	int ret = SR_ERR, i, device_count;
	uint8_t revid;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->profile->vid
		    || des.idProduct != devc->profile->pid)
			continue;
				
		if ((sdi->status == SR_ST_INITIALIZING) ||
				(sdi->status == SR_ST_INACTIVE)) {
			/*
			 * Check device by its physical USB bus/port address.
			 */		
			if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
			{
				continue;
			}

			if (strcmp(sdi->connection_id, connection_id))
			{
				/* This is not the one. */
				continue;
			}
		}
		
		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address = libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}
		
		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
						libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		ret = command_get_fw_version(usb->devhdl, &vi);
		if (ret != SR_OK) {
			sr_err("Failed to get firmware version.");
			break;
		}

		ret = command_get_revid_version(sdi, &revid);
		if (ret != SR_OK) {
			sr_err("Failed to get REVID.");
			break;
		}

		/*
		 * Changes in major version mean incompatible/API changes, so
		 * bail out if we encounter an incompatible version.
		 * Different minor versions are OK, they should be compatible.
		 */
		if (vi.major != FX3_REQUIRED_VERSION_MAJOR) {
			sr_err("Expected firmware version %d.x, "
			       "got %d.%d.", FX3_REQUIRED_VERSION_MAJOR,
			       vi.major, vi.minor);
			break;
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), "
			"interface %d, firmware %d.%d.",
			usb->bus, usb->address, connection_id,
			USB_INTERFACE, vi.major, vi.minor);

		sr_info("Detected REVID, it's a Cypress FX3!\n");

		ret = SR_OK;

		break;
	}
	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV struct dev_context *cypress_fx3_dev_new(void)
{
	struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->profile = NULL;
	devc->fw_updated = 0;
	devc->cur_samplerate = 0;
	devc->limit_frames = 1;
	devc->limit_samples = 0;
	devc->capture_ratio = 0;
	devc->sample_wide = FALSE;
	devc->num_frames = 0;
	devc->stl = NULL;

	return devc;
}

SR_PRIV void cypress_fx3_abort_acquisition(struct dev_context *devc)
{
	int i;

	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);

	/* Free the deinterlace buffers if we had them. */
	if (g_slist_length(devc->enabled_analog_channels) > 0) {
		g_free(devc->logic_buffer);
		g_free(devc->analog_buffer);
	}

	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	if (devc->submitted_transfers == 0)
		finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);

}

// retrieve and put actual samples from incoming packets
static void mso_send_data_proc(struct sr_dev_inst *sdi,
	uint8_t *data, size_t length, size_t sample_width)
{
	 
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct dev_context *devc = sdi->priv;
	struct parsed_packet pkt;
	(void)sample_width;

	size_t offset = 0;

	sr_err("mso_send_data_proc started ");

	while (offset + HEADER_SIZE <= length) {
		int parsed_len = fx3driver_parse_next_packet(&data[offset], length - offset, &pkt);

		// if(parsed_len == -3){
		// 	sr_err("Skipping to next packet %zu.", offset);
		// 	continue;;
		// }
		if (parsed_len <= 0) {
			sr_err("Invalid or incomplete packet at offset %zu.", offset);
			break;
		}

		sr_err("The parsed length of the packet is %d", parsed_len);
		// if it sees channel_type 0xFF send samples to digital channels
		if (pkt.channel_type == 0xFF) {

			int sample_width = 1; // Assuming 8-bit samples
			size_t needed_bytes = pkt.num_samples * sample_width; // we are retriveing 4 samples, each sample is 2 bytes, so the total length will be 4*2 = 8 bytes 
			if (needed_bytes > devc->logic_buffer_size) {
				devc->logic_buffer = g_realloc(devc->logic_buffer, needed_bytes);
				devc->logic_buffer_size = needed_bytes;
			}

			//memcpy(devc->logic_buffer, pkt.digital_samples, pkt.num_samples * sizeof(uint16_t));
			memcpy(devc->logic_buffer, pkt.digital_samples, needed_bytes);


			const struct sr_datafeed_logic logic = {
				.length = needed_bytes,
				.unitsize = 1,
				.data = devc->logic_buffer
			};

			const struct sr_datafeed_packet logic_packet = {
				.type = SR_DF_LOGIC,
				.payload = &logic
			};

			sr_session_send(sdi, &logic_packet);

			// Print final samples to ensure they match the expected values
			for (size_t i = 0; i < pkt.num_samples; i++) {
				printf("[FINAL] Sample[%zu] = 0x%04X\n", i, ((uint16_t *)devc->logic_buffer)[i]);
			}

		}	
		//Start processing analog samples if channel type is 0x00
		else if (pkt.channel_type == 0x00) {


			sr_err("NUMBER of CHANNELS =%zu", pkt.num_channels);
			sr_err("NUMBER of SAMPLES =%zu", pkt.num_samples);
			//size_t num_channels = devc->enabled_analog_channels;
			//size_t num_channels = NUM_ANALOG_CHANNELS;
			size_t num_channels = pkt.num_channels; 
			int sample_width = 16; // Assuming 16-bit samples
			//size_t needed_bytes = pkt.num_samples * sample_width; // we are retriveing 8 samples, each sample is 1 bytes, so the toaotl length will be 8*1 = 8 bytes 
			size_t needed_bytes = pkt.num_samples * num_channels * sizeof(float);
			// if (needed_bytes > devc->analog_buffer_size) {
			

			g_free(devc->analog_buffer);
			devc->analog_buffer = g_malloc0(needed_bytes);
			

			//devc->analog_buffer = g_realloc(devc->analog_buffer, needed_bytes);
			devc->analog_buffer_size = needed_bytes;
		


			// size_t old_size = devc->analog_buffer_size;
			// devc->analog_buffer = g_realloc(devc->analog_buffer, needed_bytes);

			// if (needed_bytes > old_size) {
			// 	memset((uint8_t *)devc->analog_buffer + old_size, 0, needed_bytes - old_size);
			// }

			// devc->analog_buffer_size = needed_bytes;


			memcpy(devc->analog_buffer, pkt.analog_samples,needed_bytes);

			//memcpy(devc->analog_buffer, pkt.analog_samples,needed_bytes);

			 /* Build a GSList with only the first num_channels enabled analog channels. */
			GSList *subset = NULL;


			GSList *it = devc->enabled_analog_channels;
			size_t ch_i = 0;
			while (it && ch_i < num_channels) {
				subset = g_slist_append(subset, it->data);  // it->data is struct sr_channel*
				it = it->next;
				ch_i++;
			}

			sr_err("overall number of samples received: %u", pkt.num_samples);
			size_t total_floats = (size_t)pkt.num_samples * num_channels;
			
			float *f = (float *)devc->analog_buffer;


			if (total_floats > 0) {
				float minv = f[0], maxv = f[0];

				for (size_t idx = 0; idx < total_floats; idx++) {
					if (f[idx] < minv) minv = f[idx];
					if (f[idx] > maxv) maxv = f[idx];
				}

				sr_err("ANALOG RANGE: min=%e max=%e over %zu floats (%u samples x %zu chans)",
					minv, maxv, total_floats,
					pkt.num_samples, num_channels);

				for (size_t idx = 0; idx < total_floats; idx++) {
					if (f[idx] < -1e6f || f[idx] > 1e6f) {
						sr_err("ANALOG BAD[%zu] = %e", idx, f[idx]);
					}
				}
			}

			size_t subset_len = g_slist_length(subset);
			sr_err("subset_len=%zu (num_channels=%zu)", subset_len, num_channels);



			sr_analog_init(&analog, &encoding, &meaning, &spec, 1);
			analog.meaning->channels = subset;
			analog.meaning->mq = SR_MQ_VOLTAGE;
			analog.meaning->unit = SR_UNIT_VOLT;
			analog.meaning->mqflags = 0 /* SR_MQFLAG_DC */;
			analog.num_samples = pkt.num_samples;
	
			analog.data = devc->analog_buffer;
			analog.encoding->is_float = true;


	

			
			sr_err("Analog scale enc=%" PRIi64 "/%" PRIi64,
      		 analog.encoding->scale.p,
      		 analog.encoding->scale.q);


		   
			sr_err("Analog meta: num_samples=%u, is_float=%d, unitsize=%u",
      		(unsigned)analog.num_samples,
      		encoding.is_float,
      		(unsigned)encoding.unitsize);
			
			
			
			sr_err("Enabled analog channels count: %zu.", (size_t)g_slist_length(devc->enabled_analog_channels));



			const struct sr_datafeed_packet analog_packet = {
				.type = SR_DF_ANALOG,
				.payload = &analog
			};


			sr_err("num_samples=%u", analog.num_samples);
			sr_err("num_channels=%zu", num_channels);
			sr_err("needed_bytes=%zu", needed_bytes);

			sr_err("SCALE DEBUG: enc_scale=%d/%d, enc_offset=%d/%d",
       		analog.encoding->scale.p,  analog.encoding->scale.q,
     		analog.encoding->offset.p, analog.encoding->offset.q);

			sr_session_send(sdi, &analog_packet);

			// Print debug
			for (size_t s = 0; s < pkt.num_samples; s++) {
				for (size_t ch = 0; ch < num_channels; ch++) {
					float v = ((float*)devc->analog_buffer)[s * num_channels + ch];
					printf("[FINAL] Sample[%zu] Channel[%zu] = %.3f V\n", s, ch, v);
				}
			}

			
		}
		offset += parsed_len;
	}
}


// Testing function to send hardcoded data
// static void mso_send_data_proc(struct sr_dev_inst *sdi,
//     uint8_t *data, size_t length, size_t sample_width)
// {
//     struct sr_datafeed_analog analog;
//     struct sr_analog_encoding encoding;
//     struct sr_analog_meaning meaning;
//     struct sr_analog_spec spec;
//     struct dev_context *devc = sdi->priv;
//     (void)sample_width;

//     sr_err("mso_send_data_proc started with fixed 8-channel test data");

//     size_t num_channels = 8;
//     size_t num_samples = 100;  

//     size_t total_floats = num_channels * num_samples;
//     size_t needed_bytes = total_floats * sizeof(float);

//     if (needed_bytes > devc->analog_buffer_size) {
//         devc->analog_buffer = g_realloc(devc->analog_buffer, needed_bytes);
//         devc->analog_buffer_size = needed_bytes;
//     }

//     float *buf = (float *)devc->analog_buffer;

//     // Fixed values per channel
//     float channel_values[8] = {
//         0.5f,
//         1.0f,
//         1.5f,
//         2.0f,
//         2.5f,
//         3.0f,
//         0.75f,
//         2.25f
//     };

//     // Fill interleaved buffer: each sample in time has the 8 channel values
//     for (size_t sample = 0; sample < num_samples; sample++) {
//         for (size_t ch = 0; ch < num_channels; ch++) {
//             buf[sample * num_channels + ch] = channel_values[ch];
//         }
//     }

//     sr_analog_init(&analog, &encoding, &meaning, &spec, num_channels);
//     analog.meaning->channels = devc->enabled_analog_channels;
//     analog.meaning->mq = SR_MQ_VOLTAGE;
//     analog.meaning->unit = SR_UNIT_VOLT;
//     analog.meaning->mqflags = 0;
//     analog.num_samples = num_samples;  // samples per channel
//     analog.data = devc->analog_buffer;

//     const struct sr_datafeed_packet analog_packet = {
//         .type = SR_DF_ANALOG,
//         .payload = &analog
//     };

//     sr_session_send(sdi, &analog_packet);

//     // Print debug
//     for (size_t i = 0; i < total_floats; i++) {
//         printf("[FINAL] Sample[%zu] = %.3f V\n", i, buf[i]);
//     }
// }


static void la_send_data_proc(struct sr_dev_inst *sdi,
	uint8_t *data, size_t length, size_t sample_width)
{
	struct dev_context *devc = sdi->priv;
	struct parsed_packet pkt;
	(void)sample_width;

	size_t offset = 0;

	sr_err("la_send_data_proc started ");
	while (offset + HEADER_SIZE <= length) {
		int parsed_len = fx3driver_parse_next_packet(&data[offset], length - offset, &pkt);

		// if(parsed_len == -3){
		// 	sr_err("Skipping to next packet %zu.", offset);
		// 	continue;;
		// }
		if (parsed_len <= 0) {
			sr_err("Invalid or incomplete packet at offset %zu.", offset);
			break;
		}

	// if it sees channel_type 0xFF send samples to digital channels
	if (pkt.channel_type == 0xFF) {

		int sample_width = 2; // Assuming 16-bit samples
		size_t needed_bytes = pkt.num_samples * sample_width; // we are retriveing 4 samples, each sample is 2 bytes, so the toaotl length will be 4*2 = 8 bytes 
		if (needed_bytes > devc->logic_buffer_size) {
			devc->logic_buffer = g_realloc(devc->logic_buffer, needed_bytes);
			devc->logic_buffer_size = needed_bytes;
		}

		memcpy(devc->logic_buffer, pkt.digital_samples, pkt.num_samples * sizeof(uint16_t));

		const struct sr_datafeed_logic logic = {
			.length = needed_bytes,
			.unitsize = 2,
			.data = devc->logic_buffer
		};

		const struct sr_datafeed_packet logic_packet = {
			.type = SR_DF_LOGIC,
			.payload = &logic
		};

		sr_session_send(sdi, &logic_packet);

		// Print final samples to ensure they match the expected values
		for (size_t i = 0; i < pkt.num_samples; i++) {
			printf("[FINAL] Sample[%zu] = 0x%04X\n", i, ((uint16_t *)devc->logic_buffer)[i]);
		}

	}


		offset += parsed_len;
	
	}
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	gboolean packet_has_error = FALSE;
	unsigned int num_samples;
	int trigger_offset, cur_sample_count, unitsize, processed_samples;
	int pre_trigger_samples;

	sdi = transfer->user_data;
	devc = sdi->priv;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	sr_err("receive_transfer(): status %s received %d bytes.",
		libusb_error_name(transfer->status), transfer->actual_length);




	/* Save incoming transfer before reusing the transfer struct. */
	//unitsize = devc->sample_wide ? 2 : 1;
	unitsize = 2; // 16-bit samples
	cur_sample_count = transfer->actual_length / unitsize;
	processed_samples = 0;



	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		cypress_fx3_abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			/*
			 * The FX3 gave up. End the acquisition, the frontend
			 * will work out that the samplecount is short.
			 */
			cypress_fx3_abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	} else {
		devc->empty_transfer_count = 0;
	}

check_trigger:
	if (devc->trigger_fired) {
		if (!devc->limit_samples || devc->sent_samples < devc->limit_samples) {
			/* Send the incoming transfer to the session bus. */
			num_samples = cur_sample_count - processed_samples;
			if (devc->limit_samples && devc->sent_samples + num_samples > devc->limit_samples)
				num_samples = devc->limit_samples - devc->sent_samples;

			devc->send_data_proc(sdi, (uint8_t *)transfer->buffer + processed_samples * unitsize,
				num_samples * unitsize, unitsize);
			devc->sent_samples += num_samples;
			processed_samples += num_samples;
		}
	} else {
		trigger_offset = soft_trigger_logic_check(devc->stl,
			transfer->buffer + processed_samples * unitsize,
			transfer->actual_length - processed_samples * unitsize,
			&pre_trigger_samples);
		if (trigger_offset > -1) {
			std_session_send_df_frame_begin(sdi);
			devc->sent_samples += pre_trigger_samples;
			num_samples = cur_sample_count - processed_samples - trigger_offset;
			if (devc->limit_samples &&
					devc->sent_samples + num_samples > devc->limit_samples)
				num_samples = devc->limit_samples - devc->sent_samples;

			devc->send_data_proc(sdi, (uint8_t *)transfer->buffer
					+ processed_samples * unitsize
					+ trigger_offset * unitsize,
					num_samples * unitsize, unitsize);
			devc->sent_samples += num_samples;
			processed_samples += trigger_offset + num_samples;

			devc->trigger_fired = TRUE;
		}
	}

	const int frame_ended = devc->limit_samples && (devc->sent_samples >= devc->limit_samples);
	const int final_frame = devc->limit_frames && (devc->num_frames >= (devc->limit_frames - 1));

	if (frame_ended) {
		devc->num_frames++;
		devc->sent_samples = 0;
		devc->trigger_fired = FALSE;
		std_session_send_df_frame_end(sdi);

		/* There may be another trigger in the remaining data, go back and check for it */
		if (processed_samples < cur_sample_count) {
			/* Reset the trigger stage */
			if (devc->stl)
				devc->stl->cur_stage = 0;
			else {
				std_session_send_df_frame_begin(sdi);
				devc->trigger_fired = TRUE;
			}
			if (!final_frame)
				goto check_trigger;
		}
	}
	if (frame_ended && final_frame) {
		cypress_fx3_abort_acquisition(devc);
		free_transfer(transfer);
	} else
		resubmit_transfer(transfer);
}





static int configure_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	const GSList *l;
	int p;
	struct sr_channel *ch;
	uint32_t channel_mask = 0, num_analog = 0;

	devc = sdi->priv;

	g_slist_free(devc->enabled_analog_channels);
	devc->enabled_analog_channels = NULL;

	for (l = sdi->channels, p = 0; l; l = l->next, p++) {
		ch = l->data;
		if ((p <= NUM_CHANNELS) && (ch->type == SR_CHANNEL_ANALOG)
				&& (ch->enabled)) {
			num_analog++;
			devc->enabled_analog_channels =
			    g_slist_append(devc->enabled_analog_channels, ch);
		} else {
			channel_mask |= ch->enabled << p;
		}
	}

	/*
	 * Use wide sampling as default for now #TODO
	 */
	devc->sample_wide = 1;

	return SR_OK;
}

static unsigned int to_bytes_per_ms(unsigned int samplerate)
{
	return samplerate / 1000;
}

static size_t get_buffer_size(struct dev_context *devc)
{
	size_t s;

	/*
	 * The buffer should be large enough to hold 10ms of data and
	 * a multiple of 512.
	 */
	s = 10 * to_bytes_per_ms(devc->cur_samplerate);
	return (s + 1023) & ~1023;
}

static unsigned int get_number_of_transfers(struct dev_context *devc)
{
	unsigned int n;

	/* Total buffer size should be able to hold about 500ms of data. */
	n = (500 * to_bytes_per_ms(devc->cur_samplerate) /
		get_buffer_size(devc));

	if (n > NUM_SIMUL_TRANSFERS)
		return NUM_SIMUL_TRANSFERS;

	return n;
}

static unsigned int get_timeout(struct dev_context *devc)
{
	size_t total_size;
	unsigned int timeout;

	total_size = get_buffer_size(devc) *
			get_number_of_transfers(devc);
	timeout = total_size / to_bytes_per_ms(devc->cur_samplerate);
	return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	drvc = (struct drv_context *)cb_data;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
}

static int start_transfers(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_trigger *trigger;
	struct libusb_transfer *transfer;
	unsigned int i, num_transfers;
	int timeout, ret;
	unsigned char *buf;
	size_t size;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;

	if ((trigger = sr_session_trigger_get(sdi->session))) {
		int pre_trigger_samples = 0;
		if (devc->limit_samples > 0)
			pre_trigger_samples = (devc->capture_ratio * devc->limit_samples) / 100;
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		devc->trigger_fired = FALSE;
	} else {
		std_session_send_df_frame_begin(sdi);
		devc->trigger_fired = TRUE;
	}

	num_transfers = get_number_of_transfers(devc);

	size = get_buffer_size(devc);
	sr_info("num_transfers: %d, buffer_size: %zu", num_transfers,size);
	devc->submitted_transfers = 0;

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	timeout = get_timeout(devc);
	devc->num_transfers = num_transfers;
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
				2 | LIBUSB_ENDPOINT_IN, buf, size,
				receive_transfer, (void *)sdi, timeout);
		sr_info("submitting transfer: %d", i);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			cypress_fx3_abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	/*
	 * If this device has analog channels and at least one of them is
	 * enabled, use mso_send_data_proc() to properly handle the analog
	 * data. Otherwise use la_send_data_proc().
	 */
	if (g_slist_length(devc->enabled_analog_channels) > 0){
		sr_err("Using mso_send_data_proc for analog channels.");
		devc->send_data_proc = mso_send_data_proc;
	}else{
		sr_err("Using la_send_data_proc for logic channels.");	
		devc->send_data_proc = la_send_data_proc;
	}
	std_session_send_df_header(sdi);

	return SR_OK;
}

SR_PRIV int cypress_fx3_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	int timeout, ret;
	size_t size;

	struct sr_usb_dev_inst *usb = sdi->conn; // sdi is your sr_dev_inst*

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;

	devc->ctx = drvc->sr_ctx;
	devc->num_frames = 0;
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	

	// sr_err("Sending CMD_RESET to FX3");
	
	// int r = libusb_control_transfer(
	// 	usb->devhdl,       // libusb_device_handle*
	// 	0x40,              // bmRequestType: Vendor, host-to-device
	// 	CMD_RESET,         // bRequest
	// 	0,                 // wValue
	// 	0,                 // wIndex
	// 	NULL,              // No data
	// 	0,                 // Length
	// 	1000               // Timeout (ms)
	// );

	// if (r < 0) {
    // sr_err("libusb_control_transfer failed: %s", libusb_error_name(r));
    // return SR_ERR;
	//}



	if (configure_channels(sdi) != SR_OK) {
		sr_err("Failed to configure channels.");
		return SR_ERR;
	}

	timeout = get_timeout(devc);

	usb_source_add(sdi->session, devc->ctx, timeout, receive_data, drvc);

	size = get_buffer_size(devc);

	
	/* Prepare for analog sampling. */
	if (g_slist_length(devc->enabled_analog_channels) > 0) {
		/* We need a buffer half the size of a transfer. */
		devc->logic_buffer = g_try_malloc(size);
		//devc->analog_buffer = g_try_malloc(sizeof(float) * size * NUM_ANALOG_CHANNELS); // mutliply by 8 because we have 8 channels
		devc->analog_buffer = g_try_malloc(sizeof(float) * size);
		if (devc->analog_buffer)
  			memset(devc->analog_buffer, 0, sizeof(float) * size);
	}
	start_transfers(sdi);
	if ((ret = command_start_acquisition(sdi)) != SR_OK) {
		cypress_fx3_abort_acquisition(devc);
		return ret;
	}

	return SR_OK;
}