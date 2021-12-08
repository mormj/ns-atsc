#include "derandomizer_cpu.hh"
#include "derandomizer_cpu_gen.hh"
#include "gnuradio/atsc/consts.hh"

namespace gr {
namespace atsc {

derandomizer_cpu::derandomizer_cpu(const block_args& args) : sync_block("derandomizer"), derandomizer(args)
{

    d_rand.reset();
}

work_return_code_t derandomizer_cpu::work(std::vector<block_work_input_sptr>& work_input,
                                  std::vector<block_work_output_sptr>& work_output)
{
    auto in = work_input[0]->items<uint8_t>();
    auto out = work_output[0]->items<uint8_t>();
    auto plin = work_input[1]->items<plinfo>();
    auto noutput_items = work_output[0]->n_items;

    for (int i = 0; i < noutput_items; i++) {
        assert(plin[i].regular_seg_p());

        if (plin[i].first_regular_seg_p())
            d_rand.reset();

        d_rand.derandomize(&out[i * ATSC_MPEG_PKT_LENGTH], &in[i * ATSC_MPEG_PKT_LENGTH]);

        // Check the pipeline info for error status and and set the
        // corresponding bit in transport packet header.

        if (plin[i].transport_error_p())
            out[i * ATSC_MPEG_PKT_LENGTH + 1] |= MPEG_TRANSPORT_ERROR_BIT;
        else
            out[i * ATSC_MPEG_PKT_LENGTH + 1] &= ~MPEG_TRANSPORT_ERROR_BIT;
    }

    produce_each(noutput_items, work_output);
    return work_return_code_t::WORK_OK;
}


} // namespace atsc
} // namespace gr