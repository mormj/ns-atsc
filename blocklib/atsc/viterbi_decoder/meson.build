#autogenerated
# This file has been automatically generated and will be overwritten by meson build
# Remove the #autogenerated comment at the top if you wish for the build scripts to leave
# this file alone
srcs = []




if IMPLEMENT_CPU
    srcs += files(['viterbi_decoder_cpu.cc'])
endif



if IMPLEMENT_CUDA
    srcs += files(['viterbi_decoder_cuda.cc'])
atsc_cu_sources += files(['viterbi_decoder.cu'])
endif

gen_srcs = custom_target('gen_viterbi_decoder_srcs',
                        input : ['viterbi_decoder.yml'],
                        output : ['viterbi_decoder.cc', 'viterbi_decoder.hh', 'viterbi_decoder_pybind.cc', 
                        'atsc_viterbi_decoder.block.yml'
                        ], 
                        command : ['python3', join_paths(SCRIPTS_DIR,'process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output-cc', '@OUTPUT0@', 
                            '--output-hh', '@OUTPUT1@', 
                            '--output-pybind', '@OUTPUT2@', 
                            '--output-grc', ['@OUTPUT3@',], 
                            '--grc-index', ['0',],
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : [false, 'include/gnuradio/atsc',false,'share/gnuradio/grc/blocks',])

gen_impls = custom_target('gen_viterbi_decoder_impl',
                        input : ['viterbi_decoder.yml'],
                        output : ['viterbi_decoder_impl_gen.hh'],
                        command : ['python3', join_paths(SCRIPTS_DIR,'process_gen_impl.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)

atsc_deps += declare_dependency(sources : [gen_srcs[1]] ) 
atsc_sources += [srcs, gen_srcs[0], gen_impls]

if get_option('enable_python') 
    

    atsc_pybind_sources += gen_srcs[2]
    atsc_pybind_names += 'viterbi_decoder'
endif