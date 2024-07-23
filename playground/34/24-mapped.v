// Benchmark "adder" written by ABC on Thu Jul 18 05:35:06 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n328, new_n329, new_n331,
    new_n332, new_n334, new_n335, new_n337, new_n338, new_n339, new_n341,
    new_n342, new_n344, new_n346, new_n347, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n04x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1d06x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  aoi012aa1n09x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1d08x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1d18x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n06x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n18x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1n20x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nona23aa1n09x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanb02aa1n12x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n112), .b(new_n113), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n107), .b(new_n117), .o1(new_n118));
  nanb03aa1n06x5               g023(.a(new_n114), .b(new_n115), .c(new_n109), .out0(new_n119));
  nor042aa1n02x5               g024(.a(new_n110), .b(new_n108), .o1(new_n120));
  norp03aa1n02x5               g025(.a(new_n119), .b(new_n113), .c(new_n120), .o1(new_n121));
  nor042aa1n06x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  inv040aa1n08x5               g027(.a(new_n114), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  nona32aa1n03x5               g029(.a(new_n118), .b(new_n124), .c(new_n122), .d(new_n121), .out0(new_n125));
  nand22aa1n06x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nor042aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand22aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n06x4               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  inv000aa1d42x5               g034(.a(new_n129), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n125), .c(new_n126), .out0(\s[10] ));
  nand23aa1n03x5               g036(.a(new_n125), .b(new_n126), .c(new_n129), .o1(new_n132));
  tech160nm_fiao0012aa1n02p5x5 g037(.a(new_n127), .b(new_n122), .c(new_n128), .o(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nor002aa1n03x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n132), .c(new_n134), .out0(\s[11] ));
  aob012aa1n03x5               g043(.a(new_n137), .b(new_n132), .c(new_n134), .out0(new_n139));
  aoi013aa1n02x4               g044(.a(new_n133), .b(new_n125), .c(new_n126), .d(new_n129), .o1(new_n140));
  tech160nm_fioaoi03aa1n03p5x5 g045(.a(\a[11] ), .b(\b[10] ), .c(new_n140), .o1(new_n141));
  norp02aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n20x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  aoib12aa1n02x5               g049(.a(new_n135), .b(new_n143), .c(new_n142), .out0(new_n145));
  aoi022aa1n02x5               g050(.a(new_n141), .b(new_n144), .c(new_n139), .d(new_n145), .o1(\s[12] ));
  inv030aa1n02x5               g051(.a(new_n124), .o1(new_n147));
  oai013aa1n03x5               g052(.a(new_n147), .b(new_n119), .c(new_n113), .d(new_n120), .o1(new_n148));
  nano23aa1n03x5               g053(.a(new_n135), .b(new_n142), .c(new_n143), .d(new_n136), .out0(new_n149));
  nano23aa1n06x5               g054(.a(new_n122), .b(new_n127), .c(new_n128), .d(new_n126), .out0(new_n150));
  nanp02aa1n03x5               g055(.a(new_n150), .b(new_n149), .o1(new_n151));
  inv000aa1n02x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n03x5               g057(.a(new_n152), .b(new_n148), .c(new_n107), .d(new_n117), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n136), .b(new_n127), .c(new_n122), .d(new_n128), .o1(new_n154));
  nona22aa1n03x5               g059(.a(new_n154), .b(new_n142), .c(new_n135), .out0(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n143), .o1(new_n156));
  nor042aa1d18x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand02aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n156), .out0(\s[13] ));
  nand42aa1n02x5               g065(.a(new_n153), .b(new_n156), .o1(new_n161));
  inv030aa1n06x5               g066(.a(new_n157), .o1(new_n162));
  nor042aa1n04x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n09x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  oaib12aa1n02x5               g069(.a(new_n162), .b(new_n163), .c(new_n164), .out0(new_n165));
  aoi012aa1n02x5               g070(.a(new_n165), .b(new_n161), .c(new_n159), .o1(new_n166));
  nano23aa1n06x5               g071(.a(new_n157), .b(new_n163), .c(new_n164), .d(new_n158), .out0(new_n167));
  nanp02aa1n02x5               g072(.a(new_n161), .b(new_n167), .o1(new_n168));
  oaoi03aa1n12x5               g073(.a(\a[14] ), .b(\b[13] ), .c(new_n162), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n163), .b(new_n168), .c(new_n170), .o1(new_n171));
  norp02aa1n02x5               g076(.a(new_n171), .b(new_n166), .o1(\s[14] ));
  nor042aa1n06x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n06x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n168), .c(new_n170), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n173), .o1(new_n177));
  aoai13aa1n03x5               g082(.a(new_n175), .b(new_n169), .c(new_n161), .d(new_n167), .o1(new_n178));
  nor042aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand02aa1d28x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n180), .o1(new_n182));
  oaih22aa1n04x5               g087(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n183));
  nona22aa1n03x5               g088(.a(new_n178), .b(new_n182), .c(new_n183), .out0(new_n184));
  aoai13aa1n03x5               g089(.a(new_n184), .b(new_n181), .c(new_n178), .d(new_n177), .o1(\s[16] ));
  nano23aa1n06x5               g090(.a(new_n173), .b(new_n179), .c(new_n180), .d(new_n174), .out0(new_n186));
  nano22aa1n03x7               g091(.a(new_n151), .b(new_n167), .c(new_n186), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n148), .c(new_n107), .d(new_n117), .o1(new_n188));
  nano22aa1n09x5               g093(.a(new_n179), .b(new_n174), .c(new_n180), .out0(new_n189));
  oai012aa1n02x5               g094(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .o1(new_n190));
  tech160nm_fioai012aa1n04x5   g095(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .o1(new_n191));
  nano23aa1n06x5               g096(.a(new_n191), .b(new_n190), .c(new_n162), .d(new_n143), .out0(new_n192));
  oab012aa1n03x5               g097(.a(new_n191), .b(new_n157), .c(new_n163), .out0(new_n193));
  ao0022aa1n03x7               g098(.a(new_n193), .b(new_n189), .c(new_n183), .d(new_n180), .o(new_n194));
  aoi013aa1n09x5               g099(.a(new_n194), .b(new_n155), .c(new_n189), .d(new_n192), .o1(new_n195));
  nanp02aa1n12x5               g100(.a(new_n188), .b(new_n195), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n186), .b(new_n169), .c(new_n161), .d(new_n167), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n197), .b(new_n180), .c(new_n183), .o1(new_n199));
  aoi022aa1n02x5               g104(.a(new_n198), .b(new_n199), .c(new_n197), .d(new_n196), .o1(\s[17] ));
  norp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand02aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  obai22aa1n02x7               g107(.a(new_n202), .b(new_n201), .c(\a[17] ), .d(\b[16] ), .out0(new_n203));
  aoi012aa1n02x5               g108(.a(new_n203), .b(new_n196), .c(new_n197), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  inv040aa1d32x5               g110(.a(\a[18] ), .o1(new_n206));
  xroi22aa1d06x4               g111(.a(new_n205), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n207));
  oai022aa1d24x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  aoi022aa1n02x5               g113(.a(new_n196), .b(new_n207), .c(new_n202), .d(new_n208), .o1(new_n209));
  oab012aa1n02x4               g114(.a(new_n204), .b(new_n209), .c(new_n201), .out0(\s[18] ));
  and002aa1n02x5               g115(.a(new_n208), .b(new_n202), .o(new_n211));
  xorc02aa1n02x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n211), .c(new_n196), .d(new_n207), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n212), .b(new_n211), .c(new_n196), .d(new_n207), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g121(.a(\a[19] ), .o1(new_n217));
  inv030aa1d32x5               g122(.a(\b[18] ), .o1(new_n218));
  nanp02aa1n04x5               g123(.a(new_n218), .b(new_n217), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[20] ), .b(\b[19] ), .out0(new_n220));
  nand02aa1d24x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  oai022aa1n03x5               g127(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n223));
  nona22aa1n03x5               g128(.a(new_n213), .b(new_n222), .c(new_n223), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n220), .c(new_n219), .d(new_n213), .o1(\s[20] ));
  inv000aa1d42x5               g130(.a(\a[20] ), .o1(new_n226));
  xroi22aa1d04x5               g131(.a(new_n217), .b(\b[18] ), .c(new_n226), .d(\b[19] ), .out0(new_n227));
  and002aa1n06x5               g132(.a(new_n227), .b(new_n207), .o(new_n228));
  nanb02aa1n12x5               g133(.a(\b[19] ), .b(new_n226), .out0(new_n229));
  oai112aa1n06x5               g134(.a(new_n229), .b(new_n221), .c(new_n218), .d(new_n217), .o1(new_n230));
  nanp03aa1d12x5               g135(.a(new_n208), .b(new_n219), .c(new_n202), .o1(new_n231));
  nand42aa1n02x5               g136(.a(new_n223), .b(new_n221), .o1(new_n232));
  oai012aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n233), .c(new_n196), .d(new_n228), .o1(new_n235));
  nano32aa1n02x4               g140(.a(new_n230), .b(new_n208), .c(new_n219), .d(new_n202), .out0(new_n236));
  nona22aa1n02x4               g141(.a(new_n232), .b(new_n236), .c(new_n234), .out0(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n196), .c(new_n228), .o1(new_n238));
  norb02aa1n03x4               g143(.a(new_n235), .b(new_n238), .out0(\s[21] ));
  inv000aa1d42x5               g144(.a(\a[21] ), .o1(new_n240));
  nanb02aa1n02x5               g145(.a(\b[20] ), .b(new_n240), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[22] ), .b(\b[21] ), .out0(new_n242));
  and002aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o(new_n243));
  oai022aa1n02x5               g148(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n244));
  nona22aa1n03x5               g149(.a(new_n235), .b(new_n243), .c(new_n244), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n242), .c(new_n241), .d(new_n235), .o1(\s[22] ));
  inv040aa1d32x5               g151(.a(\a[22] ), .o1(new_n247));
  xroi22aa1d06x4               g152(.a(new_n240), .b(\b[20] ), .c(new_n247), .d(\b[21] ), .out0(new_n248));
  and003aa1n02x5               g153(.a(new_n207), .b(new_n248), .c(new_n227), .o(new_n249));
  oaoi03aa1n02x5               g154(.a(\a[22] ), .b(\b[21] ), .c(new_n241), .o1(new_n250));
  tech160nm_fiao0012aa1n02p5x5 g155(.a(new_n250), .b(new_n233), .c(new_n248), .o(new_n251));
  xorc02aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n196), .d(new_n249), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n252), .b(new_n250), .c(new_n233), .d(new_n248), .o1(new_n254));
  aobi12aa1n02x5               g159(.a(new_n254), .b(new_n196), .c(new_n249), .out0(new_n255));
  norb02aa1n03x4               g160(.a(new_n253), .b(new_n255), .out0(\s[23] ));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .out0(new_n259));
  and002aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .o(new_n260));
  oai022aa1n02x5               g165(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n261));
  nona22aa1n03x5               g166(.a(new_n253), .b(new_n260), .c(new_n261), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n259), .c(new_n258), .d(new_n253), .o1(\s[24] ));
  inv000aa1d42x5               g168(.a(\a[23] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(\a[24] ), .o1(new_n265));
  xroi22aa1d06x4               g170(.a(new_n264), .b(\b[22] ), .c(new_n265), .d(\b[23] ), .out0(new_n266));
  and003aa1n03x7               g171(.a(new_n228), .b(new_n266), .c(new_n248), .o(new_n267));
  aoai13aa1n06x5               g172(.a(new_n266), .b(new_n250), .c(new_n233), .d(new_n248), .o1(new_n268));
  oaib12aa1n02x5               g173(.a(new_n261), .b(new_n265), .c(\b[23] ), .out0(new_n269));
  nanp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n196), .d(new_n267), .o1(new_n272));
  nanb02aa1n02x5               g177(.a(new_n271), .b(new_n269), .out0(new_n273));
  aoi122aa1n06x5               g178(.a(new_n273), .b(new_n251), .c(new_n266), .d(new_n196), .e(new_n267), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n272), .b(new_n274), .out0(\s[25] ));
  norp02aa1n02x5               g180(.a(\b[24] ), .b(\a[25] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .out0(new_n278));
  and002aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o(new_n279));
  oai022aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n280));
  nona22aa1n03x5               g185(.a(new_n272), .b(new_n279), .c(new_n280), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n277), .d(new_n272), .o1(\s[26] ));
  nanp02aa1n02x5               g187(.a(new_n278), .b(new_n271), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n248), .c(new_n266), .out0(new_n284));
  nand02aa1n02x5               g189(.a(new_n284), .b(new_n228), .o1(new_n285));
  aoi012aa1n12x5               g190(.a(new_n285), .b(new_n188), .c(new_n195), .o1(new_n286));
  aob012aa1n02x5               g191(.a(new_n280), .b(\b[25] ), .c(\a[26] ), .out0(new_n287));
  aoai13aa1n12x5               g192(.a(new_n287), .b(new_n283), .c(new_n268), .d(new_n269), .o1(new_n288));
  xorc02aa1n02x5               g193(.a(\a[27] ), .b(\b[26] ), .out0(new_n289));
  oai012aa1n06x5               g194(.a(new_n289), .b(new_n288), .c(new_n286), .o1(new_n290));
  nanb02aa1n02x5               g195(.a(new_n289), .b(new_n287), .out0(new_n291));
  aoi113aa1n02x5               g196(.a(new_n286), .b(new_n291), .c(new_n270), .d(new_n271), .e(new_n278), .o1(new_n292));
  norb02aa1n02x7               g197(.a(new_n290), .b(new_n292), .out0(\s[27] ));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .out0(new_n296));
  oai022aa1d24x5               g201(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(\a[28] ), .c(\b[27] ), .o1(new_n298));
  tech160nm_finand02aa1n05x5   g203(.a(new_n290), .b(new_n298), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n290), .d(new_n295), .o1(\s[28] ));
  xorc02aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .out0(new_n301));
  and002aa1n02x5               g206(.a(new_n296), .b(new_n289), .o(new_n302));
  oai012aa1n06x5               g207(.a(new_n302), .b(new_n288), .c(new_n286), .o1(new_n303));
  inv000aa1d42x5               g208(.a(\b[27] ), .o1(new_n304));
  oaib12aa1n09x5               g209(.a(new_n297), .b(new_n304), .c(\a[28] ), .out0(new_n305));
  nand23aa1n03x5               g210(.a(new_n303), .b(new_n305), .c(new_n301), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n305), .o1(new_n307));
  oaoi13aa1n06x5               g212(.a(new_n307), .b(new_n302), .c(new_n288), .d(new_n286), .o1(new_n308));
  oaih12aa1n02x5               g213(.a(new_n306), .b(new_n308), .c(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g215(.a(new_n289), .b(new_n301), .c(new_n296), .o(new_n311));
  tech160nm_fioaoi03aa1n03p5x5 g216(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .o1(new_n312));
  oaoi13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n288), .d(new_n286), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .out0(new_n314));
  oai012aa1n03x5               g219(.a(new_n311), .b(new_n288), .c(new_n286), .o1(new_n315));
  norb02aa1n03x5               g220(.a(new_n314), .b(new_n312), .out0(new_n316));
  nanp02aa1n02x5               g221(.a(new_n315), .b(new_n316), .o1(new_n317));
  oaih12aa1n02x5               g222(.a(new_n317), .b(new_n313), .c(new_n314), .o1(\s[30] ));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  and003aa1n02x5               g224(.a(new_n302), .b(new_n314), .c(new_n301), .o(new_n320));
  oaih12aa1n02x5               g225(.a(new_n320), .b(new_n288), .c(new_n286), .o1(new_n321));
  tech160nm_fiaoi012aa1n03p5x5 g226(.a(new_n316), .b(\a[30] ), .c(\b[29] ), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n319), .b(new_n322), .out0(new_n323));
  nanp02aa1n03x5               g228(.a(new_n321), .b(new_n323), .o1(new_n324));
  oaoi13aa1n03x5               g229(.a(new_n322), .b(new_n320), .c(new_n288), .d(new_n286), .o1(new_n325));
  oai012aa1n03x5               g230(.a(new_n324), .b(new_n325), .c(new_n319), .o1(\s[31] ));
  nano22aa1n02x4               g231(.a(new_n97), .b(new_n98), .c(new_n99), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n104), .b(new_n103), .out0(new_n328));
  nona22aa1n02x4               g233(.a(new_n104), .b(new_n103), .c(new_n97), .out0(new_n329));
  oai022aa1n02x5               g234(.a(new_n329), .b(new_n327), .c(new_n328), .d(new_n100), .o1(\s[3] ));
  nanb02aa1n02x5               g235(.a(new_n101), .b(new_n102), .out0(new_n331));
  oai012aa1n02x5               g236(.a(new_n104), .b(new_n329), .c(new_n327), .o1(new_n332));
  aboi22aa1n03x5               g237(.a(new_n101), .b(new_n107), .c(new_n332), .d(new_n331), .out0(\s[4] ));
  nona22aa1n02x4               g238(.a(new_n328), .b(new_n100), .c(new_n331), .out0(new_n334));
  norb02aa1n02x5               g239(.a(new_n111), .b(new_n110), .out0(new_n335));
  xnbna2aa1n03x5               g240(.a(new_n335), .b(new_n334), .c(new_n106), .out0(\s[5] ));
  norb02aa1n02x5               g241(.a(new_n109), .b(new_n108), .out0(new_n337));
  aoai13aa1n02x5               g242(.a(new_n337), .b(new_n110), .c(new_n107), .d(new_n111), .o1(new_n338));
  aoi112aa1n02x5               g243(.a(new_n110), .b(new_n337), .c(new_n107), .d(new_n335), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n338), .b(new_n339), .out0(\s[6] ));
  oai012aa1n02x5               g245(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n341));
  nanb02aa1n02x5               g246(.a(new_n112), .b(new_n107), .out0(new_n342));
  xobna2aa1n03x5               g247(.a(new_n116), .b(new_n342), .c(new_n341), .out0(\s[7] ));
  tech160nm_fiao0012aa1n02p5x5 g248(.a(new_n116), .b(new_n342), .c(new_n341), .o(new_n344));
  xobna2aa1n03x5               g249(.a(new_n113), .b(new_n344), .c(new_n123), .out0(\s[8] ));
  nona22aa1n02x4               g250(.a(new_n118), .b(new_n121), .c(new_n124), .out0(new_n346));
  norb02aa1n02x5               g251(.a(new_n126), .b(new_n122), .out0(new_n347));
  norp03aa1n02x5               g252(.a(new_n121), .b(new_n124), .c(new_n347), .o1(new_n348));
  aoi022aa1n02x5               g253(.a(new_n346), .b(new_n347), .c(new_n118), .d(new_n348), .o1(\s[9] ));
endmodule


