#pragma once

#include <gnuradio/atsc/sync.hh>
#include <gnuradio/atsc/consts.hh>
#include <gnuradio/filter/mmse_fir_interpolator_ff.hh>
#include <gnuradio/filter/single_pole_iir.hh>

#include <cuda.h>
#include <cuda_runtime.h>

namespace gr {
namespace atsc {

class sync_cuda : public sync
{
public:
    sync_cuda(const block_args& args);
    virtual work_return_code_t work(std::vector<block_work_input_sptr>& work_input,
                                    std::vector<block_work_output_sptr>& work_output) override;
    void reset();

private:
    gr::filter::kernel::mmse_fir_interpolator_ff d_interp;

    double d_rx_clock_to_symbol_freq;
    int d_si;
    double d_w;  // ratio of PERIOD of Tx to Rx clocks
    double d_mu; // fractional delay [0,1]
    int d_incr;

    double d_timing_adjust;
    int d_counter; // free running mod 832 counter
    int d_symbol_index;
    bool d_seg_locked;
    unsigned char d_sr; // 4 bit shift register
    signed char d_integrator[ATSC_DATA_SEGMENT_LENGTH];
    int d_output_produced;

    float *d_data_mem;
    float *d_host_in;
    float *d_host_out;
    float tmp_out[ATSC_DATA_SEGMENT_LENGTH];

    float* d_dev_in;
    float* d_dev_out;
    uint16_t* d_dev_si;
    uint16_t* d_dev_fi;
    float* d_dev_taps;

    int8_t* d_dev_integrator_accum;
    uint16_t* d_dev_corr_idx;
    int16_t* d_dev_corr_val;
    float* d_dev_timing_adjust;
    uint8_t* d_dev_sr;

    float* d_dev_params;
    float* d_host_params;

    static const int OUTPUT_MULTIPLE = 1;
    cudaStream_t d_stream;

    int d_cntr = 0;

};

} // namespace atsc
} // namespace gr