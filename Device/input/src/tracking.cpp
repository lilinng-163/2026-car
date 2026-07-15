#include "tracking.h"

int tracking::feed_byte(uint8_t byte)
{
    if (rx_buf.full())
        rx_buf.clear();

    rx_buf.push_back(byte);

    if (byte != TAIL)
        return 0;

    if (rx_buf.empty() || rx_buf.at(0) != HEAD)
    {
        sync_to_header();
        if (rx_buf.empty() || rx_buf.at(0) != HEAD)
        {
            rx_buf.clear();
            return -1;
        }
    }

    parse_frame();
    rx_buf.clear();
    return 0;
}

void tracking::sync_to_header(void)
{
    for (size_t i = 0; i < rx_buf.size(); i++)
    {
        if (rx_buf.at(i) == HEAD)
        {
            etl::vector<uint8_t, BUF_SIZE> tmp;
            for (size_t j = i; j < rx_buf.size(); j++)
            {
                if (!tmp.full())
                    tmp.push_back(rx_buf.at(j));
            }
            rx_buf = tmp;
            return;
        }
    }
    rx_buf.clear();
}

void tracking::parse_frame(void)
{
    if (rx_buf.size() < 4)
        return;

    if (rx_buf.at(0) != HEAD)
        return;

    char mode = static_cast<char>(rx_buf.at(1));
    if (mode != 'A' && mode != 'D')
        return;

    if (rx_buf.at(2) != ',')
        return;

    size_t pos = 3;

    if (mode == 'D')
    {
        for (int ch = 0; ch < 8; ch++)
        {
            while (pos < rx_buf.size() && rx_buf.at(pos) != ':')
                pos++;
            pos++;
            digital_values[ch] = (pos < rx_buf.size() && rx_buf.at(pos) == '1') ? 1 : 0;
            while (pos < rx_buf.size() && rx_buf.at(pos) != ',' && rx_buf.at(pos) != TAIL)
                pos++;
            pos++;
        }
        digital = true;
    }
    else
    {
        for (int ch = 0; ch < 8; ch++)
        {
            while (pos < rx_buf.size() && rx_buf.at(pos) != ':')
                pos++;
            pos++;
            uint16_t val = 0;
            while (pos < rx_buf.size() && rx_buf.at(pos) != ',' && rx_buf.at(pos) != TAIL)
            {
                if (rx_buf.at(pos) >= '0' && rx_buf.at(pos) <= '9')
                    val = val * 10 + static_cast<uint16_t>(rx_buf.at(pos) - '0');
                pos++;
            }
            analog_values[ch] = val;
            pos++;
        }
        digital = false;
    }
}
