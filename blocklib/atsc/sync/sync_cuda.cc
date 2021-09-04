#include "sync_cuda.hh"

#include <gnuradio/helper_cuda.h>

#include <gnuradio/filter/interpolator_taps.hh>

extern void exec_atsc_sync_and_integrate(const float* in,
                                         float* interp,
                                         float* interp_taps,
                                         int8_t* integrator_accum,
                                         float* params,
                                         cudaStream_t stream);

namespace gr {
namespace atsc {


static const double LOOP_FILTER_TAP = 0.0005; // 0.0005 works
static const double ADJUSTMENT_GAIN = 1.0e-5 / (10 * ATSC_DATA_SEGMENT_LENGTH);
static const int SYMBOL_INDEX_OFFSET = 3;
static const int MIN_SEG_LOCK_CORRELATION_VALUE = 5;
static const signed char SSI_MIN = -16;
static const signed char SSI_MAX = 15;


sync::sptr sync::make_cuda(const block_args& args)
{
    return std::make_shared<sync_cuda>(args);
}

sync_cuda::sync_cuda(const block_args& args)
    : sync(args), d_rx_clock_to_symbol_freq(args.rate / ATSC_SYMBOL_RATE), d_si(0)
{

    // checkCudaErrors(cudaMalloc(
    //     (void**)&d_dev_in,
    //     1500 + OUTPUT_MULTIPLE * sizeof(float) *
    //                (int)(ATSC_DATA_SEGMENT_LENGTH * d_rx_clock_to_symbol_freq)));

    // checkCudaErrors(cudaMallocHost(
    //     (void**)&d_host_in,
    //     1500 + OUTPUT_MULTIPLE * sizeof(float) *
    //                (int)(ATSC_DATA_SEGMENT_LENGTH * d_rx_clock_to_symbol_freq)));

    checkCudaErrors(cudaMalloc(
        (void**)&d_dev_out, OUTPUT_MULTIPLE * sizeof(float) * ATSC_DATA_SEGMENT_LENGTH));


    checkCudaErrors(cudaMalloc((void**)&d_dev_integrator_accum,
                               sizeof(int8_t) * ATSC_DATA_SEGMENT_LENGTH));

    checkCudaErrors(cudaMalloc((void**)&d_dev_params, 6 * sizeof(float)));
    checkCudaErrors(cudaMallocHost((void**)&d_host_params, 6 * sizeof(float)));

    checkCudaErrors(cudaMalloc(
        (void**)&d_data_mem, OUTPUT_MULTIPLE * ATSC_DATA_SEGMENT_LENGTH * sizeof(float)));
    // checkCudaErrors(cudaMallocHost(
    //     (void**)&d_data_mem, OUTPUT_MULTIPLE * ATSC_DATA_SEGMENT_LENGTH * sizeof(float)));

    // Copy the interpolation filter taps into device memory
    checkCudaErrors(
        cudaMalloc((void**)&d_dev_taps, sizeof(float) * (NSTEPS + 1) * NTAPS));


    cudaStreamCreate(&d_stream);

    // Taps need to be reversed into device memory
    const float* ptaps = &taps[0][0];
    std::vector<float> rev_taps(ptaps, ptaps + (NSTEPS + 1) * NTAPS);
    for (int i = 0; i < (NSTEPS + 1); i++) {
        // reverse each filter
        std::reverse(rev_taps.begin() + NTAPS * i, rev_taps.begin() + NTAPS * (i + 1));
    }

    checkCudaErrors(cudaMemcpy(d_dev_taps,
                               rev_taps.data(),
                               sizeof(float) * (NSTEPS + 1) * NTAPS,
                               cudaMemcpyHostToDevice));

    reset();

    set_output_multiple(OUTPUT_MULTIPLE);
}

void sync_cuda::reset()
{
    d_w = d_rx_clock_to_symbol_freq;
    d_mu = 0.5;

    d_timing_adjust = 0;
    d_counter = 0;
    d_symbol_index = 0;
    d_seg_locked = false;

    d_sr = 0;

    // memset(d_data_mem,
    //        0,
    //        OUTPUT_MULTIPLE * ATSC_DATA_SEGMENT_LENGTH *
    //            sizeof(*d_data_mem)); // (float)0 = 0x00000000
    checkCudaErrors(cudaMemset(
        d_data_mem, 0, OUTPUT_MULTIPLE * ATSC_DATA_SEGMENT_LENGTH * sizeof(*d_data_mem)));
    checkCudaErrors(
        cudaMemset(d_dev_integrator_accum, SSI_MIN, ATSC_DATA_SEGMENT_LENGTH));
}


work_return_code_t sync_cuda::work(std::vector<block_work_input>& work_input,
                                   std::vector<block_work_output>& work_output)
{
    auto in = static_cast<const float*>(work_input[0].items());
    auto out = static_cast<float*>(work_output[0].items());
    auto noutput_items = work_output[0].n_items;
    auto ninput_items = work_input[0].n_items;


    // Because this is a general block, we must do some forecasting
    auto min_items = static_cast<int>(noutput_items * d_rx_clock_to_symbol_freq *
                                      ATSC_DATA_SEGMENT_LENGTH) +
                     1500 - 1;
    if (ninput_items < min_items) {
        // consume_each(0,work_input);
        return work_return_code_t::WORK_INSUFFICIENT_INPUT_ITEMS;
    }
    assert(work_output[0].n_items % OUTPUT_MULTIPLE == 0);

    // int input_mem_items = ((int)(ATSC_DATA_SEGMENT_LENGTH * d_w) + 1);
    // amount actually consumed
    d_si = 0;

    // noutput items are in vectors of ATSC_DATA_SEGMENT_LENGTH
    int input_mem_items = ((int)(ATSC_DATA_SEGMENT_LENGTH * d_w) + 1);
    // amount actually consumed
    d_si = 0;

    // if (nitems_written(0) >= 1232 )
    // {
    //     volatile int x = 7;
    // }

    // std::cout << "atsc_sync: " << nitems_written(0) << std::endl;

    // noutput items are in vectors of ATSC_DATA_SEGMENT_LENGTH
    int no = 0;
    for (int n = 0; n < noutput_items; n += OUTPUT_MULTIPLE) {
        // std::cout << "atsc_sync: " << nitems_written(0) + no << std::endl;

        int d_si_start = d_si;
        double d_mu_start = d_mu;

        if ((d_si + (int)d_interp.ntaps()) >= ninput_items) {
            d_si = d_si_start;
            d_mu = d_mu_start;
            break;
        }

        // memcpy(d_host_in,
        //        in + d_si_start,
        //        OUTPUT_MULTIPLE * sizeof(float) * input_mem_items);

        // checkCudaErrors(cudaMemcpyAsync(d_dev_in,
        //                                 d_host_in,
        //                                 OUTPUT_MULTIPLE * sizeof(float) * input_mem_items,
        //                                 cudaMemcpyHostToDevice,
        //                                 d_stream));
        // cudaStreamSynchronize(d_stream);
        // Launch 832 threads to do interpolation
        // The kernel will do 8 tap dot products, so total threads / 8

        d_host_params[0] = d_timing_adjust;
        d_host_params[1] = d_mu;
        d_host_params[2] = d_w;
        d_host_params[3] = 0;

        checkCudaErrors(cudaMemcpyAsync(d_dev_params,
                                        d_host_params,
                                        sizeof(float) * 6,
                                        cudaMemcpyHostToDevice,
                                        d_stream));

        cudaStreamSynchronize(d_stream);
        // cudaDeviceSynchronize();


        for (int oo = 0; oo < OUTPUT_MULTIPLE; oo++) {
            exec_atsc_sync_and_integrate(in + d_si_start, //d_dev_in,
                                         d_dev_out + oo * ATSC_DATA_SEGMENT_LENGTH,
                                         d_dev_taps,
                                         d_dev_integrator_accum,
                                         d_dev_params,
                                         d_stream);
        }
        checkCudaErrors(cudaPeekAtLastError());
        cudaStreamSynchronize(d_stream);

        checkCudaErrors(cudaMemcpyAsync(d_host_params,
                                        d_dev_params,
                                        sizeof(float) * 6,
                                        cudaMemcpyDeviceToHost,
                                        d_stream));


        cudaStreamSynchronize(d_stream);
        d_si += (int)rint(d_host_params[3]);
        d_mu = d_host_params[1];
        d_timing_adjust = d_host_params[0];

        uint16_t tmp_idx = (uint16_t)d_host_params[4];
        int16_t tmp_val = (int16_t)d_host_params[5];

        // std::cout << d_cntr << ": "
        // << (int)rint(d_host_params[3]) << ","
        // << d_mu << ","
        // << d_host_params[0] << ","
        // << d_host_params[4] << ","
        // << d_host_params[5] << std::endl;

        d_seg_locked = tmp_val >= MIN_SEG_LOCK_CORRELATION_VALUE;


        d_cntr += OUTPUT_MULTIPLE;

        if (d_seg_locked) {

            // int idx_start = SYMBOL_INDEX_OFFSET + tmp_idx;
            int idx_start = tmp_idx - SYMBOL_INDEX_OFFSET;
            if (idx_start >= ATSC_DATA_SEGMENT_LENGTH)
                idx_start -= ATSC_DATA_SEGMENT_LENGTH;
            if (idx_start < 0)
                idx_start += ATSC_DATA_SEGMENT_LENGTH;

            // Obviously there is a double copy here that needs to be optimized
            checkCudaErrors(cudaMemcpyAsync(
                d_data_mem + (ATSC_DATA_SEGMENT_LENGTH - idx_start),
                d_dev_out,
                sizeof(float) *
                    ((OUTPUT_MULTIPLE - 1) * ATSC_DATA_SEGMENT_LENGTH + idx_start),
                cudaMemcpyDeviceToDevice,
                d_stream));
            cudaStreamSynchronize(d_stream);
            // memcpy(&out[no * ATSC_DATA_SEGMENT_LENGTH],
            //        d_data_mem,
            //        OUTPUT_MULTIPLE * ATSC_DATA_SEGMENT_LENGTH * sizeof(float));
            checkCudaErrors(cudaMemcpyAsync(&out[no * ATSC_DATA_SEGMENT_LENGTH],
                   d_data_mem,
                   OUTPUT_MULTIPLE * ATSC_DATA_SEGMENT_LENGTH * sizeof(float),
                cudaMemcpyDeviceToHost,
                d_stream));
            cudaStreamSynchronize(d_stream);
            
            checkCudaErrors(cudaMemcpyAsync(
                d_data_mem,
                d_dev_out + ATSC_DATA_SEGMENT_LENGTH * (OUTPUT_MULTIPLE - 1) + idx_start,
                sizeof(float) * (ATSC_DATA_SEGMENT_LENGTH - idx_start),
                cudaMemcpyDeviceToDevice,
                d_stream));
            cudaStreamSynchronize(d_stream);


            // std::cout << "   " << in[d_si_start] << " " << out[no] << " " << d_mu << " " << d_timing_adjust << " " << idx_start << std::endl;
            no += OUTPUT_MULTIPLE;

            
        }
    }
    consume_each(d_si, work_input);
    produce_each(no, work_output);
    return work_return_code_t::WORK_OK;
}


} // namespace atsc
} // namespace gr