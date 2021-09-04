yml_file = 'derandomizer.yml'

derandomizer_files = files(['derandomizer_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     atsc_derandomizer_files += files('derandomizer_cuda.cc')
#     atsc_cu_sources += files('derandomizer_cuda.cu')
# endif

gen_derandomizer_h = custom_target('gen_derandomizer_cpu_h',
                        input : ['derandomizer.yml'],
                        output : ['derandomizer.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_derandomizer_cc = custom_target('gen_derandomizer_cpu_cc',
                        input : ['derandomizer.yml'],
                        output : ['derandomizer.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_derandomizer_h])
atsc_sources += [derandomizer_files, gen_derandomizer_cc]

if get_option('enable_python')
    gen_derandomizer_pybind = custom_target('gen_derandomizer_cpu_pybind',
                            input : ['derandomizer.yml'],
                            output : ['derandomizer_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_derandomizer_pybind
    atsc_pybind_names += 'derandomizer'
endif
