/*
 * main.c
 *
 *  Created on: 14.08.2013
 *      Author: alexs
 *
 * Modified by Arjan Scherpenisse, july 2016
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include "rfid.h"
#include "lgpio.h"

#include <ei.h>

#define err(code, msg) (fprintf(stderr, msg "\n"), exit(code));
#define dbg(msg) (fprintf(stderr, msg "\n"));

void erlcmd_send(char *response, size_t len);
uint8_t spi_init(uint32_t spi_speed);
void send_tag(const char *uid, size_t len);

int main(int argc, char *argv[])
{

    uint8_t SN[10];
    uint16_t CType = 0;
    uint8_t SN_len = 0;
    char status;
    int tmp;

    char *p;
    char sn_str[23];

    uint32_t spi_speed = 10000000L;

    if (argc != 2)
    {
        err(1, "Usage: rc522 <spi_speed|test>");
    }

    // test mode; send tag to host every second
    if (!strcmp(argv[1], "test"))
    {
        dbg("RC522 port test mode.");
        for (;;)
        {
            send_tag("foo", 3);
            usleep(1000000);
        }
    }

    spi_speed = (uint32_t)strtoul(argv[1], NULL, 10);
    if (spi_speed > 125000L)
        spi_speed = 125000L;
    if (spi_speed < 4)
        spi_speed = 4;

    if (spi_init(spi_speed))
    {
        err(1, "SPI initialization failed.");
    }

    InitRc522();

    for (;;)
    {
        status = find_tag(&CType);
        if (status == TAG_NOTAG)
        {
            usleep(50000);
            continue;
        }
        else if ((status != TAG_OK) && (status != TAG_COLLISION))
        {
            continue;
        }

        if (select_tag_sn(SN, &SN_len) != TAG_OK)
        {
            continue;
        }

        p = sn_str;
        for (tmp = 0; tmp < SN_len; tmp++)
        {
            sprintf(p, "%02X", SN[tmp]);
            p += 2;
        }
        *p = 0;

        fprintf(stderr, "Type: %04X, Serial: %s\n", CType, &sn_str[1]);
        send_tag(sn_str, 2 * SN_len);

        PcdHalt();
    }

    cleanup();
    return 0;
}

uint8_t spi_init(uint32_t spi_speed)
{
    int h;
    h = lgGpiochipOpen(0); // Open GPIO chip 0
    if (h < 0)
    {
        dbg("Can't open GPIO chip!");
        return 1;
    }

    // Configure RST pin (GPIO25) as output
    if (lgGpioClaimOutput(h, 0, 25, 1) < 0) // Set GPIO25 high
    {
        dbg("Can't configure RST pin!");
        return 1;
    }

    int spi_handle = lgSpiOpen(0, 0, spi_speed, 0); // Open SPI on bus 0, chip select 0
    if (spi_handle < 0)
    {
        dbg("Can't open SPI device!");
        return 1;
    }

    return 0;
}

void send_tag(const char *uid, size_t len)
{
    char resp[1024];
    int resp_index = sizeof(uint16_t); // Space for payload size
    ei_encode_version(resp, &resp_index);

    ei_encode_tuple_header(resp, &resp_index, 2);
    ei_encode_atom(resp, &resp_index, "tag");
    ei_encode_binary(resp, &resp_index, uid, len);

    erlcmd_send(resp, resp_index);
}

/**
 * @brief Synchronously send a response back to Erlang
 *
 * @param response what to send back
 */
void erlcmd_send(char *response, size_t len)
{
    uint16_t be_len = htons(len - sizeof(uint16_t));
    memcpy(response, &be_len, sizeof(be_len));

    size_t wrote = 0;
    do
    {
        ssize_t amount_written = write(STDOUT_FILENO, response + wrote, len - wrote);
        if (amount_written < 0)
        {
            if (errno == EINTR)
                continue;

            // err(EXIT_FAILURE, "write");
            exit(0);
        }

        wrote += amount_written;
    } while (wrote < len);
}

void cleanup()
{
    lgSpiClose(0);      // Close SPI device
    lgGpiochipClose(0); // Close GPIO chip
}
