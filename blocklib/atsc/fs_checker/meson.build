yml_file = 'fs_checker.yml'

fs_checker_files = files(['fs_checker_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     fs_checker_files += files('fs_checker_cuda.cc')
#     atsc_cu_sources += files('fs_checker_cuda.cu')
# endif

gen_fs_checker_h = custom_target('gen_fs_checker_cpu_h',
                        input : ['fs_checker.yml'],
                        output : ['fs_checker.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_fs_checker_cc = custom_target('gen_fs_checker_cpu_cc',
                        input : ['fs_checker.yml'],
                        output : ['fs_checker.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_fs_checker_h])
atsc_sources += [fs_checker_files, gen_fs_checker_cc]

if get_option('enable_python')
    gen_fs_checker_pybind = custom_target('gen_fs_checker_cpu_pybind',
                            input : ['fs_checker.yml'],
                            output : ['fs_checker_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_fs_checker_pybind
    atsc_pybind_names += 'fs_checker'
endif
