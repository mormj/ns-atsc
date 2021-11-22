#include "deinterleaver_cpu.hh"
#include "deinterleaver_cpu_gen.hh"
#include <gnuradio/atsc/consts.hh>
#include <gnuradio/atsc/plinfo.hh>

namespace gr {
namespace atsc {

deinterleaver_cpu::deinterleaver_cpu(const block_args& args) : sync_block("deinterleaver"), deinterleaver(args), alignment_fifo(156)
{
     m_fifo.reserve(s_interleavers);

    for (int i = 0; i < s_interleavers; i++)
        m_fifo.emplace_back((s_interleavers - 1 - i) * 4);

    sync();

    set_tag_propagation_policy(tag_propagation_policy_t::TPP_CUSTOM);
}

void deinterleaver_cpu::reset()
{
    sync();

    for (auto& i : m_fifo)
        i.reset();
}


work_return_code_t deinterleaver_cpu::work(std::vector<block_work_input>& work_input,
                                  std::vector<block_work_output>& work_output)
{
    auto in = work_input[0].items<uint8_t>();
    auto out = work_output[0].items<uint8_t>();
    auto plin = work_input[1].items<plinfo>();
    auto plout = work_output[1].items<plinfo>();
    auto noutput_items = work_output[0].n_items;

    for (int i = 0; i < noutput_items; i++) {
        assert(plin[i].regular_seg_p());

        // reset commutator if required using INPUT pipeline info
        if (plin[i].first_regular_seg_p())
            sync();

        // remap OUTPUT pipeline info to reflect all data segment end-to-end delay
        plinfo::delay(plout[i], plin[i], s_interleavers);

        // now do the actual deinterleaving
        for (unsigned int j = 0; j < ATSC_MPEG_RS_ENCODED_LENGTH; j++) {
            out[i * ATSC_MPEG_RS_ENCODED_LENGTH + j] =
                alignment_fifo.stuff(transform(in[i * ATSC_MPEG_RS_ENCODED_LENGTH + j]));
        }
    }

    produce_each(noutput_items, work_output);
    return work_return_code_t::WORK_OK;
}


} // namespace atsc
} // namespace gr