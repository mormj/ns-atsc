/* -*- c++ -*- */
/*
 * Copyright 2014 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <gnuradio/atsc/randomizer_lfsr.hh>

namespace gr {
namespace atsc {

unsigned char randomizer_lfsr::s_output_map[1 << 14];
bool randomizer_lfsr::s_output_map_initialized_p = false;

randomizer_lfsr::randomizer_lfsr()
{
    d_state = PRELOAD_VALUE;

    if (!s_output_map_initialized_p)
        initialize_output_map();
}

/*!
 * \brief Generate the table used in the fast_output_map function.
 *
 * The table has 16K byte entries, but because of how is is used, only
 * 256 entries end up being resident in the cache.  This seems
 * like a good use of memory.  We can get away with a 16K table
 * because the low two bits of the state do not affect the output
 * function.  By shifting right those two bits we shrink the table,
 * and also get better cache line utilization.
 */
void randomizer_lfsr::initialize_output_map()
{
    s_output_map_initialized_p = true;

    for (int i = 0; i < (1 << 14); i++)
        s_output_map[i] = slow_output_map(i << 2);
}


void randomizer_lfsr::reset() { d_state = PRELOAD_VALUE; }

void randomizer_lfsr::randomize(atsc_mpeg_packet_no_sync& out, const atsc_mpeg_packet& in)
{
    assert(in.data[0] == MPEG_SYNC_BYTE); // confirm it's there, then drop

    for (int i = 0; i < ATSC_MPEG_DATA_LENGTH; i++)
        out.data[i] = in.data[i + 1] ^ output_and_clk();
}

void randomizer_lfsr::derandomize(uint8_t* out, const uint8_t* in)
{
    out[0] = MPEG_SYNC_BYTE; // add sync byte to beginning of packet

    for (int i = 0; i < ATSC_MPEG_DATA_LENGTH; i++)
        out[i + 1] = in[i] ^ output_and_clk();
}


unsigned char randomizer_lfsr::slow_output_map(int st)
{
    int output = 0;

    if (st & 0x8000)
        output |= 0x01;

    if (st & 0x2000)
        output |= 0x02;

    if (st & 0x1000)
        output |= 0x04;

    if (st & 0x0200)
        output |= 0x08;

    if (st & 0x0020)
        output |= 0x10;

    if (st & 0x0010)
        output |= 0x20;

    if (st & 0x0008)
        output |= 0x40;

    if (st & 0x0004)
        output |= 0x80;

    return output;
}

} /* namespace atsc */
} /* namespace gr */
