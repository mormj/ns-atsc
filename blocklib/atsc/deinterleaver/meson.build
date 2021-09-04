yml_file = 'deinterleaver.yml'

deinterleaver_files = files(['deinterleaver_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     deinterleaver_files += files('deinterleaver_cuda.cc')
#     atsc_cu_sources += files('deinterleaver_cuda.cu')
# endif

gen_deinterleaver_h = custom_target('gen_deinterleaver_cpu_h',
                        input : ['deinterleaver.yml'],
                        output : ['deinterleaver.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_deinterleaver_cc = custom_target('gen_deinterleaver_cpu_cc',
                        input : ['deinterleaver.yml'],
                        output : ['deinterleaver.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_deinterleaver_h])
atsc_sources += [deinterleaver_files, gen_deinterleaver_cc]

if get_option('enable_python')
    gen_deinterleaver_pybind = custom_target('gen_deinterleaver_cpu_pybind',
                            input : ['deinterleaver.yml'],
                            output : ['deinterleaver_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_deinterleaver_pybind
    atsc_pybind_names += 'deinterleaver'
endif
