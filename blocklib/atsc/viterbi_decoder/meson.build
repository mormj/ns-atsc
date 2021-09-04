yml_file = 'atsc_viterbi_decoder.yml'

viterbi_decoder_files = files(['viterbi_decoder_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     viterbi_decoder_files += files('viterbi_decoder_cuda.cc')
#     atsc_cu_sources += files('viterbi_decoder_cuda.cu')
# endif

gen_viterbi_decoder_h = custom_target('gen_viterbi_decoder_cpu_h',
                        input : ['viterbi_decoder.yml'],
                        output : ['viterbi_decoder.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_viterbi_decoder_cc = custom_target('gen_viterbi_decoder_cpu_cc',
                        input : ['viterbi_decoder.yml'],
                        output : ['viterbi_decoder.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_viterbi_decoder_h])
atsc_sources += [viterbi_decoder_files, gen_viterbi_decoder_cc]

if get_option('enable_python')
    gen_viterbi_decoder_pybind = custom_target('gen_viterbi_decoder_cpu_pybind',
                            input : ['viterbi_decoder.yml'],
                            output : ['viterbi_decoder_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_viterbi_decoder_pybind
    atsc_pybind_names += 'viterbi_decoder'
endif
