yml_file = 'rs_decoder.yml'

rs_decoder_files = files(['rs_decoder_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     rs_decoder_files += files('rs_decoder_cuda.cc')
#     atsc_cu_sources += files('rs_decoder_cuda.cu')
# endif

gen_rs_decoder_h = custom_target('gen_rs_decoder_cpu_h',
                        input : ['rs_decoder.yml'],
                        output : ['rs_decoder.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_rs_decoder_cc = custom_target('gen_rs_decoder_cpu_cc',
                        input : ['rs_decoder.yml'],
                        output : ['rs_decoder.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_rs_decoder_h])
atsc_sources += [rs_decoder_files, gen_rs_decoder_cc]

if get_option('enable_python')
    gen_rs_decoder_pybind = custom_target('gen_rs_decoder_cpu_pybind',
                            input : ['rs_decoder.yml'],
                            output : ['rs_decoder_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_rs_decoder_pybind
    atsc_pybind_names += 'rs_decoder'
endif
