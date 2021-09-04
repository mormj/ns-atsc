yml_file = 'fpll.yml'

fpll_files = files(['fpll_cpu.cc'])


# if cuda_dep.found() and get_option('enable_cuda')
#     fpll_files += files('fpll_cuda.cc')
#     cu_sources += files('fpll_cuda.cu')
# endif

gen_fpll_h = custom_target('gen_fpll_cpu_h',
                        input : ['fpll.yml'],
                        output : ['fpll.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_fpll_cc = custom_target('gen_fpll_cpu_cc',
                        input : ['fpll.yml'],
                        output : ['fpll.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_fpll_h])
atsc_sources += [fpll_files, gen_fpll_cc]

if get_option('enable_python')
    gen_fpll_pybind = custom_target('gen_fpll_cpu_pybind',
                            input : ['fpll.yml'],
                            output : ['fpll_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_fpll_pybind
    atsc_pybind_names += 'fpll'
endif
