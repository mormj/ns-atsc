#include "fs_checker_cpu.hh"
#include "fs_checker_cpu_gen.hh"

#include "pnXXX.hh"
#include "types.hh"
#include <gnuradio/atsc/consts.hh>
#include <gnuradio/atsc/fs_checker.hh>
#include <string>

#define ATSC_SEGMENTS_PER_DATA_FIELD 313

static const int PN511_ERROR_LIMIT = 20; // max number of bits wrong
static const int PN63_ERROR_LIMIT = 5;

namespace gr {
namespace atsc {

fs_checker_cpu::fs_checker_cpu(const block_args& args) : block("fs_checker"), fs_checker(args)
{
    reset();
}

void fs_checker_cpu::reset()
{
    d_index = 0;
    memset(d_sample_sr, 0, sizeof(d_sample_sr));
    memset(d_tag_sr, 0, sizeof(d_tag_sr));
    memset(d_bit_sr, 0, sizeof(d_bit_sr));
    d_field_num = 0;
    d_segment_num = 0;
}

work_return_code_t fs_checker_cpu::work(std::vector<block_work_input_sptr>& work_input,
                                             std::vector<block_work_output_sptr>& work_output)
{
    auto in = work_input[0]->items<float>();
    auto out = work_output[0]->items<float>();
    auto plout = work_output[1]->items<plinfo>();
    auto noutput_items = work_output[0]->n_items;
    auto ninput_items = work_input[0]->n_items;

    // Need to figure out how to handle this more gracefully
    // The scheduler (currently) has no information about what the block
    // is doing and doesn't know to give ninput >= noutput
    if (ninput_items < noutput_items)
    {
        return work_return_code_t::WORK_INSUFFICIENT_INPUT_ITEMS;
    }

    // std::cout << noutput_items << "/" << ninput_items << std::endl;

    int output_produced = 0;

    for (int i = 0; i < noutput_items; i++) {
        // check for a hit on the PN 511 pattern
        int errors = 0;

        for (int j = 0; j < LENGTH_511 && errors < PN511_ERROR_LIMIT; j++)
            errors +=
                (in[i * ATSC_DATA_SEGMENT_LENGTH + j + OFFSET_511] >= 0) ^ pn511[j];

        GR_LOG_DEBUG(_debug_logger,
                     std::string("second PN63 error count = ") + std::to_string(errors));

        if (errors < PN511_ERROR_LIMIT) { // 511 pattern is good.
            // determine if this is field 1 or field 2
            errors = 0;
            for (int j = 0; j < LENGTH_2ND_63; j++)
                errors += (in[i * ATSC_DATA_SEGMENT_LENGTH + j + OFFSET_2ND_63] >= 0) ^
                          pn63[j];

            // we should have either field 1 (== PN63) or field 2 (== ~PN63)
            if (errors <= PN63_ERROR_LIMIT) {
                GR_LOG_DEBUG(_debug_logger, "Found FIELD_SYNC_1")
                d_field_num = 1;    // We are in field number 1 now
                d_segment_num = -1; // This is the first segment
            } else if (errors >= (LENGTH_2ND_63 - PN63_ERROR_LIMIT)) {
                GR_LOG_DEBUG(_debug_logger, "Found FIELD_SYNC_2")
                d_field_num = 2;    // We are in field number 2 now
                d_segment_num = -1; // This is the first segment
            } else {
                // should be extremely rare.
                GR_LOG_WARN(_logger,
                            std::string("PN63 error count = ") + std::to_string(errors));
            }
        }

        if (d_field_num == 1 || d_field_num == 2) { // If we have sync
            // So we copy out current packet data to an output packet and fill its plinfo
            memcpy(&out[output_produced * ATSC_DATA_SEGMENT_LENGTH],
                   &in[i * ATSC_DATA_SEGMENT_LENGTH],
                   ATSC_DATA_SEGMENT_LENGTH * sizeof(float));

            plinfo pli_out;
            pli_out.set_regular_seg((d_field_num == 2), d_segment_num);

            d_segment_num++;
            if (d_segment_num > (ATSC_SEGMENTS_PER_DATA_FIELD - 1)) {
                d_field_num = 0;
                d_segment_num = 0;
            } else {
                plout[output_produced++] = pli_out;                
            }
        }
    }

    consume_each(noutput_items,work_input);
    produce_each(output_produced,work_output);
    return work_return_code_t::WORK_OK;
}


} // namespace atsc
} // namespace gr