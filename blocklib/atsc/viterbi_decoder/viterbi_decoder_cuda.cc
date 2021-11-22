#include "viterbi_decoder_cuda.hh"
#include "viterbi_decoder_cuda_gen.hh"

#include <gnuradio/atsc/plinfo.hh>

#include <gnuradio/helper_cuda.h>

extern void exec_deinterleave_kernel(const float* in, float* out, cudaStream_t stream);
extern void exec_viterbi_kernel(const float* in,
                                unsigned char* out,
                                float* path_metrics,
                                unsigned long long* traceback,
                                int* post_coder_state,
                                cudaStream_t stream);


extern void
exec_interleave_kernel(unsigned char* in, unsigned char* out, cudaStream_t stream);

namespace gr {
namespace atsc {

viterbi_decoder_cuda::viterbi_decoder_cuda(const block_args& args) : sync_block("viterbi_decoder"), viterbi_decoder(args)
{
    set_output_multiple(NCODERS); // TODO - how to handle

    set_tag_propagation_policy(tag_propagation_policy_t::TPP_CUSTOM);

    set_output_multiple(NCODERS);


    checkCudaErrors(
        cudaMalloc((void**)&d_data, sizeof(float) * NCODERS * ATSC_DATA_SEGMENT_LENGTH));

    checkCudaErrors(cudaMallocHost((void**)&d_host_in,
                                   sizeof(float) * NCODERS * ATSC_DATA_SEGMENT_LENGTH));

    checkCudaErrors(cudaMallocHost((void**)&d_host_out, OUTPUT_SIZE));

    checkCudaErrors(cudaMalloc((void**)&d_dibits,
                               sizeof(unsigned char) * NCODERS * (enco_which_max + 797)));

    checkCudaErrors(
        cudaMalloc((void**)&d_out_copy, sizeof(unsigned char) * (OUTPUT_SIZE)));

    checkCudaErrors(cudaMalloc((void**)&d_path_metrics, sizeof(float) * NCODERS * 2 * 4));

    checkCudaErrors(cudaMemset(d_path_metrics, 0, sizeof(float) * NCODERS * 2 * 4));


    checkCudaErrors(
        cudaMalloc((void**)&d_traceback, sizeof(unsigned long long) * NCODERS * 2 * 4));

    checkCudaErrors(
        cudaMemset(d_traceback, 0, sizeof(unsigned long long) * NCODERS * 2 * 4));

    checkCudaErrors(
        cudaMalloc((void**)&d_post_coder_state, sizeof(unsigned char) * NCODERS));

    checkCudaErrors(cudaMemset(d_post_coder_state, 0, sizeof(unsigned char) * NCODERS));

    cudaStreamCreate(&d_stream);
}

void viterbi_decoder_cuda::reset()
{

}

work_return_code_t viterbi_decoder_cuda::work(std::vector<block_work_input>& work_input,
                                              std::vector<block_work_output>& work_output)
{
    auto in = work_input[0].items<float>();
    auto out = work_output[0].items<uint8_t>();
    auto plin = work_input[1].items<plinfo>();
    auto plout = work_output[1].items<plinfo>();

    auto noutput_items = work_output[0].n_items;
    // The way the fs_checker works ensures we start getting packets
    // starting with a field sync, and our input multiple is set to
    // 12, so we should always get a mod 12 numbered first packet
    assert(noutput_items % NCODERS == 0);


    // force it to be only 1 noutput item for now
    // noutput_items = NCODERS;
    for (int i = 0; i < noutput_items; i += NCODERS) {

        exec_deinterleave_kernel(in + i * ATSC_DATA_SEGMENT_LENGTH, d_data, d_stream);

        exec_viterbi_kernel(d_data,
                            d_dibits,
                            d_path_metrics,
                            d_traceback,
                            d_post_coder_state,
                            d_stream);

        exec_interleave_kernel(
            d_dibits, &out[i * ATSC_MPEG_RS_ENCODED_LENGTH], d_stream);

        // checkCudaErrors(cudaMemcpyAsync(d_host_out,
        //                            &d_out_copy[0],
        //                            sizeof(unsigned char) * ATSC_MPEG_RS_ENCODED_LENGTH
        //                            * NCODERS, cudaMemcpyDeviceToHost, d_stream));


        // copy output from contiguous temp buffer into final output
        for (int j = 0; j < NCODERS; j++) {
            plinfo::delay(plout[i + j], plin[i + j], NCODERS);
        }

        // cudaStreamSynchronize(d_stream);
        // memcpy(&out[i * ATSC_MPEG_RS_ENCODED_LENGTH], d_host_out, OUTPUT_SIZE);
    }

    cudaStreamSynchronize(d_stream);
    produce_each(noutput_items, work_output);
    return work_return_code_t::WORK_OK;
}


} // namespace atsc
} // namespace gr