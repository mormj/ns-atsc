#include "fpll_cpu.hh"
#include "fpll_cpu_gen.hh"

namespace gr {
namespace atsc {

fpll_cpu::fpll_cpu(const block_args& args) : sync_block("fpll"), fpll(args)
{
    d_afc.set_taps(1.0 - exp(-1.0 / args.rate / 5e-6));
    d_nco.set_freq((-3e6 + 0.309e6) / args.rate * 2 * GR_M_PI);
    d_nco.set_phase(0.0);
}

work_return_code_t fpll_cpu::work(std::vector<block_work_input_sptr>& work_input,
                                  std::vector<block_work_output_sptr>& work_output)
{
    constexpr float alpha = 0.01;
    constexpr float beta = alpha * alpha / 4.0;

    auto in = work_input[0]->items<gr_complex>();
    auto out = work_output[0]->items<float>();
    auto noutput_items = work_output[0]->n_items;

    float a_cos, a_sin;
    float x;
    gr_complex result, filtered;

    for (int k = 0; k < noutput_items; k++) {
        d_nco.step();                 // increment phase
        d_nco.sincos(&a_sin, &a_cos); // compute cos and sin

        // Mix out carrier and output I-only signal
        gr::fast_cc_multiply(result, in[k], gr_complex(a_sin, a_cos));

        out[k] = result.real();

        // Update phase/freq error
        filtered = d_afc.filter(result);
        x = gr::fast_atan2f(filtered.imag(), filtered.real());

        // std::cout << out[k] << "/" << x << std::endl;

        // avoid slamming filter with big transitions
        if (x > M_PI_2)
            x = M_PI_2;
        else if (x < -M_PI_2)
            x = -M_PI_2;

        d_nco.adjust_phase(alpha * x);
        d_nco.adjust_freq(beta * x);
    }

    work_output[0]->produce(noutput_items);
    return work_return_code_t::WORK_OK;

}


} // namespace atsc
} // namespace gr