yml_file = 'equalizer.yml'

equalizer_files = files(['equalizer_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     equalizer_files += files('equalizer_cuda.cc')
#     cu_sources += files('equalizer_cuda.cu')
# endif

gen_equalizer_h = custom_target('gen_equalizer_cpu_h',
                        input : ['equalizer.yml'],
                        output : ['equalizer.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_equalizer_cc = custom_target('gen_equalizer_cpu_cc',
                        input : ['equalizer.yml'],
                        output : ['equalizer.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_equalizer_h])
atsc_sources += [equalizer_files, gen_equalizer_cc]

if get_option('enable_python')
    gen_equalizer_pybind = custom_target('gen_equalizer_cpu_pybind',
                            input : ['equalizer.yml'],
                            output : ['equalizer_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_equalizer_pybind
    atsc_pybind_names += 'equalizer'
endif
