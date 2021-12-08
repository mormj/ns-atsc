#pragma once

#include <gnuradio/atsc/derandomizer.hh>

#include "randomizer_lfsr.hh"

namespace gr {
namespace atsc {

class derandomizer_cpu : public derandomizer
{
public:
    derandomizer_cpu(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;

private:
    randomizer_lfsr d_rand;
};

} // namespace atsc
} // namespace gr