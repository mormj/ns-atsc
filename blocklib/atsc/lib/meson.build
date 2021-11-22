incdir = include_directories(['../include/gnuradio/atsc','../include'])

atsc_deps += [newsched_runtime_dep, newsched_blocklib_filter_dep, newsched_blocklib_math_dep, volk_dep, fmt_dep]

atsc_sources += [
    'plinfo.cc',
    'single_viterbi.cc',
    'randomizer_lfsr.cc',
]


# This is ugly, and there HAS to be a better way to bring in dependencies
link_args = ['-lnewsched-runtime','-lnewsched-blocklib-filter','-lnewsched-blocklib-math','-lnewsched-blocklib-fec']
block_cpp_args = ['-DHAVE_CPU']
if USE_CUDA
     block_cpp_args += '-DHAVE_CUDA'

    newsched_blocklib_atsc_cu = library('newsched-blocklib-atsc-cu', 
        atsc_cu_sources, 
        include_directories : incdir, 
        install : true, 
        dependencies : [cuda_dep])

    newsched_blocklib_atsc_cu_dep = declare_dependency(include_directories : incdir,
                        link_with : newsched_blocklib_atsc_cu,
                        dependencies : cuda_dep)

    atsc_deps += [cuda_dep, cusp_dep, newsched_blocklib_atsc_cu_dep]
    link_args += '-lcusp'
endif

incdir = include_directories(['../include/gnuradio/atsc','../include'])
newsched_blocklib_atsc_lib = library('newsched-blocklib-atsc', 
    atsc_sources, 
    include_directories : incdir, 
    install : true,
    link_language: 'cpp',
    link_args : link_args,  # why is this necesary???
    dependencies : atsc_deps,
    cpp_args : block_cpp_args)

newsched_blocklib_atsc_dep = declare_dependency(include_directories : incdir,
					   link_with : newsched_blocklib_atsc_lib,
                       dependencies : atsc_deps)
