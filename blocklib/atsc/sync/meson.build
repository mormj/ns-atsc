yml_file = 'sync.yml'

sync_files = files(['sync_cpu.cc'])


if USE_CUDA
    sync_files += files('sync_cuda.cc')
    atsc_cu_sources += files('sync.cu')
endif

gen_sync_h = custom_target('gen_sync_cpu_h',
                        input : ['sync.yml'],
                        output : ['sync.hh'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : true,
                        install_dir : 'include/gnuradio/atsc')

gen_sync_cc = custom_target('gen_sync_cpu_cc',
                        input : ['sync.yml'],
                        output : ['sync.cc'],
                        command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_folder.py'),
                            '--yaml_file', '@INPUT@', 
                            '--output_file', '@OUTPUT@', 
                            '--build_dir', join_paths(meson.build_root())],
                        install : false)        

atsc_deps += declare_dependency(sources : [gen_sync_h])
atsc_sources += [sync_files, gen_sync_cc]

if get_option('enable_python')
    gen_sync_pybind = custom_target('gen_sync_cpu_pybind',
                            input : ['sync.yml'],
                            output : ['sync_pybind.cc'],
                            command : ['python3', join_paths(share_dir,'utils/blockbuilder/scripts/process_pybind.py'),
                                '--yaml_file', '@INPUT@', 
                                '--output_file', '@OUTPUT@', 
                                '--build_dir', join_paths(meson.build_root())],
                            install : false)                   
    atsc_pybind_sources += gen_sync_pybind
    atsc_pybind_names += 'sync'
endif
