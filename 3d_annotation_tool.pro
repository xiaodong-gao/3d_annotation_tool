QT       += core gui opengl concurrent svg printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

msvc {
    QMAKE_CFLAGS += /utf-8
    QMAKE_CXXFLAGS += /utf-8
}

win32{
    #pcl 1.14.1 root
    3DPARTY_DIR = D:/PCL1141
    INCLUDEPATH += $$3DPARTY_DIR/include/pcl-1.14
    INCLUDEPATH += $$3DPARTY_DIR/3rdParty/Boost/include/boost-1_84
    INCLUDEPATH += $$3DPARTY_DIR/3rdParty/VTK/include/vtk-9.3
    INCLUDEPATH += $$3DPARTY_DIR/3rdParty/Eigen3/include/eigen3


    CONFIG(debug, debug|release){
        LIBS += -L$$3DPARTY_DIR/lib -lpcl_commond \
                                    -lpcl_featuresd \
                                    -lpcl_filtersd \
                                    -lpcl_io_plyd \
                                    -lpcl_iod \
                                    -lpcl_kdtreed \
                                    -lpcl_keypointsd \
                                    -lpcl_mld \
                                    -lpcl_octreed \
                                    -lpcl_outofcored \
                                    -lpcl_peopled \
                                    -lpcl_recognitiond \
                                    -lpcl_registrationd \
                                    -lpcl_sample_consensusd \
                                    -lpcl_searchd \
                                    -lpcl_segmentationd \
                                    -lpcl_stereod \
                                    -lpcl_surfaced \
                                    -lpcl_trackingd \
                                    -lpcl_visualizationd

        LIBS += -L$$3DPARTY_DIR/3rdParty/Boost/lib   -llibboost_atomic-vc143-mt-gd-x64-1_84 \
                                            -llibboost_bzip2-vc143-mt-gd-x64-1_84 \
                                            -llibboost_chrono-vc143-mt-gd-x64-1_84 \
                                            -llibboost_container-vc143-mt-gd-x64-1_84 \
                                            -llibboost_context-vc143-mt-gd-x64-1_84 \
                                            -llibboost_contract-vc143-mt-gd-x64-1_84 \
                                            -llibboost_coroutine-vc143-mt-gd-x64-1_84 \
                                            -llibboost_date_time-vc143-mt-gd-x64-1_84 \
                                            -llibboost_exception-vc143-mt-gd-x64-1_84 \
                                            -llibboost_fiber-vc143-mt-gd-x64-1_84 \
                                            -llibboost_filesystem-vc143-mt-gd-x64-1_84 \
                                            -llibboost_graph-vc143-mt-gd-x64-1_84 \
                                            -llibboost_graph_parallel-vc143-mt-gd-x64-1_84 \
                                            -llibboost_iostreams-vc143-mt-gd-x64-1_84 \
                                            -llibboost_json-vc143-mt-gd-x64-1_84 \
                                            -llibboost_locale-vc143-mt-gd-x64-1_84 \
                                            -llibboost_log-vc143-mt-gd-x64-1_84 \
                                            -llibboost_log_setup-vc143-mt-gd-x64-1_84 \
                                            -llibboost_math_c99-vc143-mt-gd-x64-1_84 \
                                            -llibboost_math_c99f-vc143-mt-gd-x64-1_84 \
                                            -llibboost_math_c99l-vc143-mt-gd-x64-1_84 \
                                            -llibboost_math_tr1-vc143-mt-gd-x64-1_84 \
                                            -llibboost_math_tr1f-vc143-mt-gd-x64-1_84 \
                                            -llibboost_math_tr1l-vc143-mt-gd-x64-1_84 \
                                            -llibboost_mpi-vc143-mt-gd-x64-1_84 \
                                            -llibboost_nowide-vc143-mt-gd-x64-1_84 \
                                            -llibboost_numpy310-vc143-mt-gd-x64-1_84 \
                                            -llibboost_prg_exec_monitor-vc143-mt-gd-x64-1_84 \
                                            -llibboost_program_options-vc143-mt-gd-x64-1_84 \
                                            -llibboost_python310-vc143-mt-gd-x64-1_84 \
                                            -llibboost_random-vc143-mt-gd-x64-1_84 \
                                            -llibboost_regex-vc143-mt-gd-x64-1_84 \
                                            -llibboost_serialization-vc143-mt-gd-x64-1_84 \
                                            -llibboost_stacktrace_noop-vc143-mt-gd-x64-1_84 \
                                            -llibboost_stacktrace_windbg-vc143-mt-gd-x64-1_84 \
                                            -llibboost_stacktrace_windbg_cached-vc143-mt-gd-x64-1_84 \
                                            -llibboost_system-vc143-mt-gd-x64-1_84 \
                                            -llibboost_test_exec_monitor-vc143-mt-gd-x64-1_84 \
                                            -llibboost_thread-vc143-mt-gd-x64-1_84 \
                                            -llibboost_timer-vc143-mt-gd-x64-1_84 \
                                            -llibboost_type_erasure-vc143-mt-gd-x64-1_84 \
                                            -llibboost_unit_test_framework-vc143-mt-gd-x64-1_84 \
                                            -llibboost_url-vc143-mt-gd-x64-1_84 \
                                            -llibboost_wave-vc143-mt-gd-x64-1_84 \
                                            -llibboost_wserialization-vc143-mt-gd-x64-1_84 \
                                            -llibboost_zlib-vc143-mt-gd-x64-1_84


            LIBS += -L$$3DPARTY_DIR/3rdParty/VTK/lib -lvtkcgns-9.3-gd \
                                                    -lvtkChartsCore-9.3-gd \
                                                    -lvtkCommonColor-9.3-gd \
                                                    -lvtkCommonComputationalGeometry-9.3-gd \
                                                    -lvtkCommonCore-9.3-gd \
                                                    -lvtkCommonDataModel-9.3-gd \
                                                    -lvtkCommonExecutionModel-9.3-gd \
                                                    -lvtkCommonMath-9.3-gd \
                                                    -lvtkCommonMisc-9.3-gd \
                                                    -lvtkCommonSystem-9.3-gd \
                                                    -lvtkCommonTransforms-9.3-gd \
                                                    -lvtkDICOMParser-9.3-gd \
                                                    -lvtkDomainsChemistry-9.3-gd \
                                                    -lvtkDomainsChemistryOpenGL2-9.3-gd \
                                                    -lvtkdoubleconversion-9.3-gd \
                                                    -lvtkexodusII-9.3-gd \
                                                    -lvtkexpat-9.3-gd \
                                                    -lvtkFiltersAMR-9.3-gd \
                                                    -lvtkFiltersCellGrid-9.3-gd \
                                                    -lvtkFiltersCore-9.3-gd \
                                                    -lvtkFiltersExtraction-9.3-gd \
                                                    -lvtkFiltersFlowPaths-9.3-gd \
                                                    -lvtkFiltersGeneral-9.3-gd \
                                                    -lvtkFiltersGeneric-9.3-gd \
                                                    -lvtkFiltersGeometry-9.3-gd \
                                                    -lvtkFiltersGeometryPreview-9.3-gd \
                                                    -lvtkFiltersHybrid-9.3-gd \
                                                    -lvtkFiltersHyperTree-9.3-gd \
                                                    -lvtkFiltersImaging-9.3-gd \
                                                    -lvtkFiltersModeling-9.3-gd \
                                                    -lvtkFiltersParallel-9.3-gd \
                                                    -lvtkFiltersParallelImaging-9.3-gd \
                                                    -lvtkFiltersPoints-9.3-gd \
                                                    -lvtkFiltersProgrammable-9.3-gd \
                                                    -lvtkFiltersReduction-9.3-gd \
                                                    -lvtkFiltersSelection-9.3-gd \
                                                    -lvtkFiltersSMP-9.3-gd \
                                                    -lvtkFiltersSources-9.3-gd \
                                                    -lvtkFiltersStatistics-9.3-gd \
                                                    -lvtkFiltersTensor-9.3-gd \
                                                    -lvtkFiltersTexture-9.3-gd \
                                                    -lvtkFiltersTopology-9.3-gd \
                                                    -lvtkFiltersVerdict-9.3-gd \
                                                    -lvtkfmt-9.3-gd \
                                                    -lvtkfreetype-9.3-gd \
                                                    -lvtkGeovisCore-9.3-gd \
                                                    -lvtkgl2ps-9.3-gd \
                                                    -lvtkglew-9.3-gd \
                                                    -lvtkGUISupportQt-9.3-gd \
                                                    -lvtkGUISupportQtQuick-9.3-gd \
                                                    -lvtkGUISupportQtSQL-9.3-gd \
                                                    -lvtkhdf5-9.3-gd \
                                                    -lvtkhdf5_hl-9.3-gd \
                                                    -lvtkImagingColor-9.3-gd \
                                                    -lvtkImagingCore-9.3-gd \
                                                    -lvtkImagingFourier-9.3-gd \
                                                    -lvtkImagingGeneral-9.3-gd \
                                                    -lvtkImagingHybrid-9.3-gd \
                                                    -lvtkImagingMath-9.3-gd \
                                                    -lvtkImagingMorphological-9.3-gd \
                                                    -lvtkImagingSources-9.3-gd \
                                                    -lvtkImagingStatistics-9.3-gd \
                                                    -lvtkImagingStencil-9.3-gd \
                                                    -lvtkInfovisCore-9.3-gd \
                                                    -lvtkInfovisLayout-9.3-gd \
                                                    -lvtkInteractionImage-9.3-gd \
                                                    -lvtkInteractionStyle-9.3-gd \
                                                    -lvtkInteractionWidgets-9.3-gd \
                                                    -lvtkIOAMR-9.3-gd \
                                                    -lvtkIOAsynchronous-9.3-gd \
                                                    -lvtkIOCellGrid-9.3-gd \
                                                    -lvtkIOCesium3DTiles-9.3-gd \
                                                    -lvtkIOCGNSReader-9.3-gd \
                                                    -lvtkIOChemistry-9.3-gd \
                                                    -lvtkIOCityGML-9.3-gd \
                                                    -lvtkIOCONVERGECFD-9.3-gd \
                                                    -lvtkIOCore-9.3-gd \
                                                    -lvtkIOEnSight-9.3-gd \
                                                    -lvtkIOExodus-9.3-gd \
                                                    -lvtkIOExport-9.3-gd \
                                                    -lvtkIOExportGL2PS-9.3-gd \
                                                    -lvtkIOExportPDF-9.3-gd \
                                                    -lvtkIOFLUENTCFF-9.3-gd \
                                                    -lvtkIOGeometry-9.3-gd \
                                                    -lvtkIOHDF-9.3-gd \
                                                    -lvtkIOImage-9.3-gd \
                                                    -lvtkIOImport-9.3-gd \
                                                    -lvtkIOInfovis-9.3-gd \
                                                    -lvtkIOIOSS-9.3-gd \
                                                    -lvtkIOLegacy-9.3-gd \
                                                    -lvtkIOLSDyna-9.3-gd \
                                                    -lvtkIOMINC-9.3-gd \
                                                    -lvtkIOMotionFX-9.3-gd \
                                                    -lvtkIOMovie-9.3-gd \
                                                    -lvtkIONetCDF-9.3-gd \
                                                    -lvtkIOOggTheora-9.3-gd \
                                                    -lvtkIOParallel-9.3-gd \
                                                    -lvtkIOParallelXML-9.3-gd \
                                                    -lvtkIOPLY-9.3-gd \
                                                    -lvtkIOSegY-9.3-gd \
                                                    -lvtkIOSQL-9.3-gd \
                                                    -lvtkioss-9.3-gd \
                                                    -lvtkIOTecplotTable-9.3-gd \
                                                    -lvtkIOVeraOut-9.3-gd \
                                                    -lvtkIOVideo-9.3-gd \
                                                    -lvtkIOXML-9.3-gd \
                                                    -lvtkIOXMLParser-9.3-gd \
                                                    -lvtkjpeg-9.3-gd \
                                                    -lvtkjsoncpp-9.3-gd \
                                                    -lvtkkissfft-9.3-gd \
                                                    -lvtklibharu-9.3-gd \
                                                    -lvtklibproj-9.3-gd \
                                                    -lvtklibxml2-9.3-gd \
                                                    -lvtkloguru-9.3-gd \
                                                    -lvtklz4-9.3-gd \
                                                    -lvtklzma-9.3-gd \
                                                    -lvtkmetaio-9.3-gd \
                                                    -lvtknetcdf-9.3-gd \
                                                    -lvtkogg-9.3-gd \
                                                    -lvtkParallelCore-9.3-gd \
                                                    -lvtkParallelDIY-9.3-gd \
                                                    -lvtkpng-9.3-gd \
                                                    -lvtkpugixml-9.3-gd \
                                                    -lvtkRenderingAnnotation-9.3-gd \
                                                    -lvtkRenderingCellGrid-9.3-gd \
                                                    -lvtkRenderingContext2D-9.3-gd \
                                                    -lvtkRenderingContextOpenGL2-9.3-gd \
                                                    -lvtkRenderingCore-9.3-gd \
                                                    -lvtkRenderingFreeType-9.3-gd \
                                                    -lvtkRenderingGL2PSOpenGL2-9.3-gd \
                                                    -lvtkRenderingHyperTreeGrid-9.3-gd \
                                                    -lvtkRenderingImage-9.3-gd \
                                                    -lvtkRenderingLabel-9.3-gd \
                                                    -lvtkRenderingLICOpenGL2-9.3-gd \
                                                    -lvtkRenderingLOD-9.3-gd \
                                                    -lvtkRenderingOpenGL2-9.3-gd \
                                                    -lvtkRenderingQt-9.3-gd \
                                                    -lvtkRenderingSceneGraph-9.3-gd \
                                                    -lvtkRenderingUI-9.3-gd \
                                                    -lvtkRenderingVolume-9.3-gd \
                                                    -lvtkRenderingVolumeOpenGL2-9.3-gd \
                                                    -lvtkRenderingVtkJS-9.3-gd \
                                                    -lvtksqlite-9.3-gd \
                                                    -lvtksys-9.3-gd \
                                                    -lvtkTestingRendering-9.3-gd \
                                                    -lvtktheora-9.3-gd \
                                                    -lvtktiff-9.3-gd \
                                                    -lvtkverdict-9.3-gd \
                                                    -lvtkViewsContext2D-9.3-gd \
                                                    -lvtkViewsCore-9.3-gd \
                                                    -lvtkViewsInfovis-9.3-gd \
                                                    -lvtkViewsQt-9.3-gd \
                                                    -lvtkWrappingTools-9.3-gd \
                                                    -lvtkzlib-9.3-gd
    }


    CONFIG(release, debug|release){
        # release output information
        DEFINES += QT_MESSAGELOGCONTEXT
        LIBS += -L$$3DPARTY_DIR/lib -lpcl_common \
                                    -lpcl_features \
                                    -lpcl_filters \
                                    -lpcl_io \
                                    -lpcl_io_ply \
                                    -lpcl_kdtree \
                                    -lpcl_keypoints \
                                    -lpcl_ml \
                                    -lpcl_octree \
                                    -lpcl_outofcore \
                                    -lpcl_people \
                                    -lpcl_recognition \
                                    -lpcl_registration \
                                    -lpcl_sample_consensus \
                                    -lpcl_search \
                                    -lpcl_segmentation \
                                    -lpcl_stereo \
                                    -lpcl_surface \
                                    -lpcl_tracking \
                                    -lpcl_visualization

        LIBS += -L$$3DPARTY_DIR/Boost/lib   -llibboost_atomic-vc143-mt-x64-1_84 \
                                            -llibboost_bzip2-vc143-mt-x64-1_84 \
                                            -llibboost_chrono-vc143-mt-x64-1_84 \
                                            -llibboost_container-vc143-mt-x64-1_84 \
                                            -llibboost_context-vc143-mt-x64-1_84 \
                                            -llibboost_contract-vc143-mt-x64-1_84 \
                                            -llibboost_coroutine-vc143-mt-x64-1_84 \
                                            -llibboost_date_time-vc143-mt-x64-1_84 \
                                            -llibboost_exception-vc143-mt-x64-1_84 \
                                            -llibboost_fiber-vc143-mt-x64-1_84 \
                                            -llibboost_filesystem-vc143-mt-x64-1_84 \
                                            -llibboost_graph-vc143-mt-x64-1_84 \
                                            -llibboost_graph_parallel-vc143-mt-x64-1_84 \
                                            -llibboost_iostreams-vc143-mt-x64-1_84 \
                                            -llibboost_json-vc143-mt-x64-1_84 \
                                            -llibboost_locale-vc143-mt-x64-1_84 \
                                            -llibboost_log-vc143-mt-x64-1_84 \
                                            -llibboost_log_setup-vc143-mt-x64-1_84 \
                                            -llibboost_math_c99-vc143-mt-x64-1_84 \
                                            -llibboost_math_c99f-vc143-mt-x64-1_84 \
                                            -llibboost_math_c99l-vc143-mt-x64-1_84 \
                                            -llibboost_math_tr1-vc143-mt-x64-1_84 \
                                            -llibboost_math_tr1f-vc143-mt-x64-1_84 \
                                            -llibboost_math_tr1l-vc143-mt-x64-1_84 \
                                            -llibboost_mpi-vc143-mt-x64-1_84 \
                                            -llibboost_nowide-vc143-mt-x64-1_84 \
                                            -llibboost_numpy310-vc143-mt-x64-1_84 \
                                            -llibboost_prg_exec_monitor-vc143-mt-x64-1_84 \
                                            -llibboost_program_options-vc143-mt-x64-1_84 \
                                            -llibboost_python310-vc143-mt-x64-1_84 \
                                            -llibboost_random-vc143-mt-x64-1_84 \
                                            -llibboost_regex-vc143-mt-x64-1_84 \
                                            -llibboost_serialization-vc143-mt-x64-1_84 \
                                            -llibboost_stacktrace_noop-vc143-mt-x64-1_84 \
                                            -llibboost_stacktrace_windbg-vc143-mt-x64-1_84 \
                                            -llibboost_stacktrace_windbg_cached-vc143-mt-x64-1_84 \
                                            -llibboost_system-vc143-mt-x64-1_84 \
                                            -llibboost_test_exec_monitor-vc143-mt-x64-1_84 \
                                            -llibboost_thread-vc143-mt-x64-1_84 \
                                            -llibboost_timer-vc143-mt-x64-1_84 \
                                            -llibboost_type_erasure-vc143-mt-x64-1_84 \
                                            -llibboost_unit_test_framework-vc143-mt-x64-1_84 \
                                            -llibboost_url-vc143-mt-x64-1_84 \
                                            -llibboost_wave-vc143-mt-x64-1_84 \
                                            -llibboost_wserialization-vc143-mt-x64-1_84 \
                                            -llibboost_zlib-vc143-mt-x64-1_84


            LIBS += -L$$3DPARTY_DIR/3rdParty/VTK/lib -lvtkcgns-9.3 \
                                                    -lvtkChartsCore-9.3 \
                                                    -lvtkCommonColor-9.3 \
                                                    -lvtkCommonComputationalGeometry-9.3 \
                                                    -lvtkCommonCore-9.3 \
                                                    -lvtkCommonDataModel-9.3 \
                                                    -lvtkCommonExecutionModel-9.3 \
                                                    -lvtkCommonMath-9.3 \
                                                    -lvtkCommonMisc-9.3 \
                                                    -lvtkCommonSystem-9.3 \
                                                    -lvtkCommonTransforms-9.3 \
                                                    -lvtkDICOMParser-9.3 \
                                                    -lvtkDomainsChemistry-9.3 \
                                                    -lvtkDomainsChemistryOpenGL2-9.3 \
                                                    -lvtkdoubleconversion-9.3 \
                                                    -lvtkexodusII-9.3 \
                                                    -lvtkexpat-9.3 \
                                                    -lvtkFiltersAMR-9.3 \
                                                    -lvtkFiltersCellGrid-9.3 \
                                                    -lvtkFiltersCore-9.3 \
                                                    -lvtkFiltersExtraction-9.3 \
                                                    -lvtkFiltersFlowPaths-9.3 \
                                                    -lvtkFiltersGeneral-9.3 \
                                                    -lvtkFiltersGeneric-9.3 \
                                                    -lvtkFiltersGeometry-9.3 \
                                                    -lvtkFiltersGeometryPreview-9.3 \
                                                    -lvtkFiltersHybrid-9.3 \
                                                    -lvtkFiltersHyperTree-9.3 \
                                                    -lvtkFiltersImaging-9.3 \
                                                    -lvtkFiltersModeling-9.3 \
                                                    -lvtkFiltersParallel-9.3 \
                                                    -lvtkFiltersParallelImaging-9.3 \
                                                    -lvtkFiltersPoints-9.3 \
                                                    -lvtkFiltersProgrammable-9.3 \
                                                    -lvtkFiltersReduction-9.3 \
                                                    -lvtkFiltersSelection-9.3 \
                                                    -lvtkFiltersSMP-9.3 \
                                                    -lvtkFiltersSources-9.3 \
                                                    -lvtkFiltersStatistics-9.3 \
                                                    -lvtkFiltersTensor-9.3 \
                                                    -lvtkFiltersTexture-9.3 \
                                                    -lvtkFiltersTopology-9.3 \
                                                    -lvtkFiltersVerdict-9.3 \
                                                    -lvtkfmt-9.3 \
                                                    -lvtkfreetype-9.3 \
                                                    -lvtkGeovisCore-9.3 \
                                                    -lvtkgl2ps-9.3 \
                                                    -lvtkglew-9.3 \
                                                    -lvtkGUISupportQt-9.3 \
                                                    -lvtkGUISupportQtQuick-9.3 \
                                                    -lvtkGUISupportQtSQL-9.3 \
                                                    -lvtkhdf5-9.3 \
                                                    -lvtkhdf5_hl-9.3 \
                                                    -lvtkImagingColor-9.3 \
                                                    -lvtkImagingCore-9.3 \
                                                    -lvtkImagingFourier-9.3 \
                                                    -lvtkImagingGeneral-9.3 \
                                                    -lvtkImagingHybrid-9.3 \
                                                    -lvtkImagingMath-9.3 \
                                                    -lvtkImagingMorphological-9.3 \
                                                    -lvtkImagingSources-9.3 \
                                                    -lvtkImagingStatistics-9.3 \
                                                    -lvtkImagingStencil-9.3 \
                                                    -lvtkInfovisCore-9.3 \
                                                    -lvtkInfovisLayout-9.3 \
                                                    -lvtkInteractionImage-9.3 \
                                                    -lvtkInteractionStyle-9.3 \
                                                    -lvtkInteractionWidgets-9.3 \
                                                    -lvtkIOAMR-9.3 \
                                                    -lvtkIOAsynchronous-9.3 \
                                                    -lvtkIOCellGrid-9.3 \
                                                    -lvtkIOCesium3DTiles-9.3 \
                                                    -lvtkIOCGNSReader-9.3 \
                                                    -lvtkIOChemistry-9.3 \
                                                    -lvtkIOCityGML-9.3 \
                                                    -lvtkIOCONVERGECFD-9.3 \
                                                    -lvtkIOCore-9.3 \
                                                    -lvtkIOEnSight-9.3 \
                                                    -lvtkIOExodus-9.3 \
                                                    -lvtkIOExport-9.3 \
                                                    -lvtkIOExportGL2PS-9.3 \
                                                    -lvtkIOExportPDF-9.3 \
                                                    -lvtkIOFLUENTCFF-9.3 \
                                                    -lvtkIOGeometry-9.3 \
                                                    -lvtkIOHDF-9.3 \
                                                    -lvtkIOImage-9.3 \
                                                    -lvtkIOImport-9.3 \
                                                    -lvtkIOInfovis-9.3 \
                                                    -lvtkIOIOSS-9.3 \
                                                    -lvtkIOLegacy-9.3 \
                                                    -lvtkIOLSDyna-9.3 \
                                                    -lvtkIOMINC-9.3 \
                                                    -lvtkIOMotionFX-9.3 \
                                                    -lvtkIOMovie-9.3 \
                                                    -lvtkIONetCDF-9.3 \
                                                    -lvtkIOOggTheora-9.3 \
                                                    -lvtkIOParallel-9.3 \
                                                    -lvtkIOParallelXML-9.3 \
                                                    -lvtkIOPLY-9.3 \
                                                    -lvtkIOSegY-9.3 \
                                                    -lvtkIOSQL-9.3 \
                                                    -lvtkioss-9.3 \
                                                    -lvtkIOTecplotTable-9.3 \
                                                    -lvtkIOVeraOut-9.3 \
                                                    -lvtkIOVideo-9.3 \
                                                    -lvtkIOXML-9.3 \
                                                    -lvtkIOXMLParser-9.3 \
                                                    -lvtkjpeg-9.3 \
                                                    -lvtkjsoncpp-9.3 \
                                                    -lvtkkissfft-9.3 \
                                                    -lvtklibharu-9.3 \
                                                    -lvtklibproj-9.3 \
                                                    -lvtklibxml2-9.3 \
                                                    -lvtkloguru-9.3 \
                                                    -lvtklz4-9.3 \
                                                    -lvtklzma-9.3 \
                                                    -lvtkmetaio-9.3 \
                                                    -lvtknetcdf-9.3 \
                                                    -lvtkogg-9.3 \
                                                    -lvtkParallelCore-9.3 \
                                                    -lvtkParallelDIY-9.3 \
                                                    -lvtkpng-9.3 \
                                                    -lvtkpugixml-9.3 \
                                                    -lvtkRenderingAnnotation-9.3 \
                                                    -lvtkRenderingCellGrid-9.3 \
                                                    -lvtkRenderingContext2D-9.3 \
                                                    -lvtkRenderingContextOpenGL2-9.3 \
                                                    -lvtkRenderingCore-9.3 \
                                                    -lvtkRenderingFreeType-9.3 \
                                                    -lvtkRenderingGL2PSOpenGL2-9.3 \
                                                    -lvtkRenderingHyperTreeGrid-9.3 \
                                                    -lvtkRenderingImage-9.3 \
                                                    -lvtkRenderingLabel-9.3 \
                                                    -lvtkRenderingLICOpenGL2-9.3 \
                                                    -lvtkRenderingLOD-9.3 \
                                                    -lvtkRenderingOpenGL2-9.3 \
                                                    -lvtkRenderingQt-9.3 \
                                                    -lvtkRenderingSceneGraph-9.3 \
                                                    -lvtkRenderingUI-9.3 \
                                                    -lvtkRenderingVolume-9.3 \
                                                    -lvtkRenderingVolumeOpenGL2-9.3 \
                                                    -lvtkRenderingVtkJS-9.3 \
                                                    -lvtksqlite-9.3 \
                                                    -lvtksys-9.3 \
                                                    -lvtkTestingRendering-9.3 \
                                                    -lvtktheora-9.3 \
                                                    -lvtktiff-9.3 \
                                                    -lvtkverdict-9.3 \
                                                    -lvtkViewsContext2D-9.3 \
                                                    -lvtkViewsCore-9.3 \
                                                    -lvtkViewsInfovis-9.3 \
                                                    -lvtkViewsQt-9.3 \
                                                    -lvtkWrappingTools-9.3 \
                                                    -lvtkzlib-9.3
    }
}



# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Annotaion.cpp \
    main.cpp  \
    pcl/visualization/MyCloudLUT.cpp \
    pcl/visualization/pcl_visualizer_extented.cpp \
    project/SystemManager.cpp \
    view/CCPushButton.cpp \
    view/flowlayout.cpp \
    visualizer.cpp \
    vtkAnnotationBoxSource.cpp \
    vtkBoxWidgetCallback.cpp \
    vtkBoxWidgetRestricted.cpp

HEADERS += \
    Annotaion.h \
    common.h \
    pcl/visualization/MyCloudLUT.h \
    pcl/visualization/PointCloudColorHandlerLUT.h \
    pcl/visualization/pcl_visualizer_extented.h \
    project/SystemManager.h \
    view/CCPushButton.h \
    view/flowlayout.h \
    visualizer.h \
    vtkAnnotationBoxSource.h \
    vtkBoxWidgetCallback.h \
    vtkBoxWidgetRestricted.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
