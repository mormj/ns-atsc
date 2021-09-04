#pragma once

#include <gnuradio/atsc/viterbi_decoder.hh>

#include <gnuradio/atsc/interleaver_fifo.hh>
#include <gnuradio/atsc/consts.hh>
#include "types.hh"
#include "viterbi_mux.hh"
#include "single_viterbi.hh"

namespace gr {
namespace atsc {

class viterbi_decoder_cpu : public viterbi_decoder
{
public:
    viterbi_decoder_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input>& work_input,
                                    std::vector<block_work_output>& work_output) override;

    std::vector<float> decoder_metrics() const;

    void reset();

private:
    static const int NCODERS = 12;
    typedef interleaver_fifo<unsigned char> fifo_t;

    static constexpr int SEGMENT_SIZE = ATSC_MPEG_RS_ENCODED_LENGTH; // 207
    static constexpr int OUTPUT_SIZE = (SEGMENT_SIZE * 12);
    static constexpr int INPUT_SIZE = (ATSC_DATA_SEGMENT_LENGTH * 12);

    single_viterbi viterbi[NCODERS];
    std::vector<fifo_t> fifo;

};

} // namespace atsc
} // namespace gr