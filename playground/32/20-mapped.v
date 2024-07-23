// Benchmark "adder" written by ABC on Thu Jul 18 04:31:12 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n329, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n354, new_n356, new_n357, new_n358, new_n360,
    new_n361, new_n362, new_n364, new_n365, new_n367, new_n368, new_n369,
    new_n370, new_n371, new_n373, new_n375, new_n376, new_n377;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  aoi022aa1d24x5               g003(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n99));
  xorc02aa1n12x5               g004(.a(\a[3] ), .b(\b[2] ), .out0(new_n100));
  oai022aa1d18x5               g005(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n101));
  oaoi13aa1n12x5               g006(.a(new_n101), .b(new_n100), .c(new_n98), .d(new_n99), .o1(new_n102));
  inv020aa1d32x5               g007(.a(\a[6] ), .o1(new_n103));
  inv040aa1d32x5               g008(.a(\b[5] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  and002aa1n12x5               g010(.a(\b[6] ), .b(\a[7] ), .o(new_n106));
  aoi112aa1n09x5               g011(.a(new_n106), .b(new_n105), .c(new_n103), .d(new_n104), .o1(new_n107));
  nanp02aa1n06x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  aob012aa1d15x5               g013(.a(new_n108), .b(\b[5] ), .c(\a[6] ), .out0(new_n109));
  nand22aa1n06x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  oai012aa1n12x5               g015(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .o1(new_n111));
  nor002aa1n12x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norb02aa1n06x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  nona23aa1d18x5               g019(.a(new_n107), .b(new_n114), .c(new_n111), .d(new_n109), .out0(new_n115));
  aoi112aa1n03x5               g020(.a(new_n106), .b(new_n105), .c(\a[8] ), .d(\b[7] ), .o1(new_n116));
  and002aa1n12x5               g021(.a(\b[5] ), .b(\a[6] ), .o(new_n117));
  oai022aa1n04x7               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  norb03aa1n03x5               g023(.a(new_n118), .b(new_n117), .c(new_n112), .out0(new_n119));
  aoi012aa1n02x5               g024(.a(new_n112), .b(new_n105), .c(new_n113), .o1(new_n120));
  aobi12aa1n12x5               g025(.a(new_n120), .b(new_n119), .c(new_n116), .out0(new_n121));
  oai012aa1d24x5               g026(.a(new_n121), .b(new_n102), .c(new_n115), .o1(new_n122));
  xnrc02aa1n12x5               g027(.a(\b[8] ), .b(\a[9] ), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n123), .o1(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n97), .c(new_n122), .d(new_n124), .o1(new_n126));
  aoi112aa1n02x5               g031(.a(new_n125), .b(new_n97), .c(new_n122), .d(new_n124), .o1(new_n127));
  norb02aa1n02x7               g032(.a(new_n126), .b(new_n127), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv030aa1d32x5               g034(.a(\b[9] ), .o1(new_n130));
  oaoi03aa1n02x5               g035(.a(new_n129), .b(new_n130), .c(new_n97), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(new_n130), .b(new_n129), .o1(new_n132));
  nand42aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nano22aa1n03x7               g038(.a(new_n123), .b(new_n132), .c(new_n133), .out0(new_n134));
  nanp02aa1n03x5               g039(.a(new_n122), .b(new_n134), .o1(new_n135));
  xorc02aa1n12x5               g040(.a(\a[11] ), .b(\b[10] ), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n135), .c(new_n131), .out0(\s[11] ));
  aob012aa1n03x5               g042(.a(new_n136), .b(new_n135), .c(new_n131), .out0(new_n138));
  nor002aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  and002aa1n12x5               g044(.a(\b[11] ), .b(\a[12] ), .o(new_n140));
  nor042aa1n02x5               g045(.a(new_n140), .b(new_n139), .o1(new_n141));
  nor002aa1n04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  oab012aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n139), .out0(new_n143));
  tech160nm_fioai012aa1n03p5x5 g048(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .o1(new_n144));
  aoi022aa1n02x7               g049(.a(new_n144), .b(new_n141), .c(new_n138), .d(new_n143), .o1(\s[12] ));
  nano32aa1n03x7               g050(.a(new_n123), .b(new_n141), .c(new_n125), .d(new_n136), .out0(new_n146));
  aoi012aa1n02x7               g051(.a(new_n97), .b(new_n129), .c(new_n130), .o1(new_n147));
  aoi112aa1n06x5               g052(.a(new_n140), .b(new_n139), .c(\a[11] ), .d(\b[10] ), .o1(new_n148));
  nona23aa1n06x5               g053(.a(new_n148), .b(new_n133), .c(new_n147), .d(new_n142), .out0(new_n149));
  aoib12aa1n09x5               g054(.a(new_n139), .b(new_n142), .c(new_n140), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n149), .b(new_n150), .o1(new_n151));
  xorc02aa1n02x5               g056(.a(\a[13] ), .b(\b[12] ), .out0(new_n152));
  aoai13aa1n03x5               g057(.a(new_n152), .b(new_n151), .c(new_n122), .d(new_n146), .o1(new_n153));
  nano22aa1n02x4               g058(.a(new_n152), .b(new_n149), .c(new_n150), .out0(new_n154));
  aobi12aa1n02x5               g059(.a(new_n154), .b(new_n146), .c(new_n122), .out0(new_n155));
  norb02aa1n02x7               g060(.a(new_n153), .b(new_n155), .out0(\s[13] ));
  inv040aa1d30x5               g061(.a(\a[13] ), .o1(new_n157));
  nanb02aa1n12x5               g062(.a(\b[12] ), .b(new_n157), .out0(new_n158));
  xorc02aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n158), .out0(\s[14] ));
  inv040aa1n12x5               g065(.a(\a[14] ), .o1(new_n161));
  xroi22aa1d06x4               g066(.a(new_n157), .b(\b[12] ), .c(new_n161), .d(\b[13] ), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n151), .c(new_n122), .d(new_n146), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  nor002aa1d32x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanp02aa1n04x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norb02aa1n03x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n163), .c(new_n165), .out0(\s[15] ));
  aob012aa1n03x5               g074(.a(new_n168), .b(new_n163), .c(new_n165), .out0(new_n170));
  xorc02aa1n02x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  inv040aa1d32x5               g076(.a(\a[16] ), .o1(new_n172));
  inv040aa1n18x5               g077(.a(\b[15] ), .o1(new_n173));
  nand22aa1n12x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  nanp02aa1n12x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n166), .b(new_n174), .c(new_n175), .o1(new_n176));
  inv030aa1n02x5               g081(.a(new_n166), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n168), .o1(new_n178));
  aoai13aa1n02x7               g083(.a(new_n177), .b(new_n178), .c(new_n163), .d(new_n165), .o1(new_n179));
  aoi022aa1n03x5               g084(.a(new_n179), .b(new_n171), .c(new_n170), .d(new_n176), .o1(\s[16] ));
  inv000aa1n02x5               g085(.a(new_n174), .o1(new_n181));
  nano32aa1n06x5               g086(.a(new_n181), .b(new_n177), .c(new_n175), .d(new_n167), .out0(new_n182));
  nand02aa1d06x5               g087(.a(new_n162), .b(new_n182), .o1(new_n183));
  nano32aa1d15x5               g088(.a(new_n183), .b(new_n134), .c(new_n136), .d(new_n141), .out0(new_n184));
  nand02aa1n06x5               g089(.a(new_n184), .b(new_n122), .o1(new_n185));
  xorc02aa1n12x5               g090(.a(\a[17] ), .b(\b[16] ), .out0(new_n186));
  nanp03aa1n02x5               g091(.a(new_n174), .b(new_n167), .c(new_n175), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[13] ), .b(\a[14] ), .o1(new_n188));
  oai022aa1d18x5               g093(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n189));
  oai112aa1n02x7               g094(.a(new_n189), .b(new_n188), .c(\b[14] ), .d(\a[15] ), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n186), .b(new_n181), .c(new_n175), .d(new_n166), .o1(new_n191));
  oai012aa1n02x5               g096(.a(new_n191), .b(new_n190), .c(new_n187), .o1(new_n192));
  aoib12aa1n02x5               g097(.a(new_n192), .b(new_n151), .c(new_n183), .out0(new_n193));
  aob012aa1n02x5               g098(.a(new_n174), .b(new_n166), .c(new_n175), .out0(new_n194));
  oab012aa1n06x5               g099(.a(new_n194), .b(new_n190), .c(new_n187), .out0(new_n195));
  aoai13aa1n12x5               g100(.a(new_n195), .b(new_n183), .c(new_n149), .d(new_n150), .o1(new_n196));
  nanb02aa1n09x5               g101(.a(new_n196), .b(new_n185), .out0(new_n197));
  aoi022aa1n02x7               g102(.a(new_n197), .b(new_n186), .c(new_n193), .d(new_n185), .o1(\s[17] ));
  inv000aa1d42x5               g103(.a(\b[16] ), .o1(new_n199));
  oaib12aa1n06x5               g104(.a(new_n197), .b(new_n199), .c(\a[17] ), .out0(new_n200));
  inv000aa1d42x5               g105(.a(\a[17] ), .o1(new_n201));
  oaib12aa1n06x5               g106(.a(new_n200), .b(\b[16] ), .c(new_n201), .out0(new_n202));
  nor042aa1d18x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nanp02aa1n24x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  norb02aa1n06x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aboi22aa1n03x5               g110(.a(new_n203), .b(new_n204), .c(new_n201), .d(new_n199), .out0(new_n206));
  aoi022aa1n02x7               g111(.a(new_n202), .b(new_n205), .c(new_n200), .d(new_n206), .o1(\s[18] ));
  and002aa1n02x5               g112(.a(new_n186), .b(new_n205), .o(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n196), .c(new_n122), .d(new_n184), .o1(new_n209));
  nor042aa1d18x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  oa0012aa1n02x5               g115(.a(new_n204), .b(new_n203), .c(new_n210), .o(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nor002aa1d32x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand02aa1d28x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n06x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n209), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g122(.a(new_n215), .b(new_n209), .c(new_n212), .out0(new_n218));
  nor002aa1d24x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand02aa1d28x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n06x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  aboi22aa1n03x5               g128(.a(new_n219), .b(new_n220), .c(new_n222), .d(new_n223), .out0(new_n224));
  inv000aa1n09x5               g129(.a(new_n213), .o1(new_n225));
  inv040aa1n03x5               g130(.a(new_n215), .o1(new_n226));
  aoai13aa1n02x7               g131(.a(new_n225), .b(new_n226), .c(new_n209), .d(new_n212), .o1(new_n227));
  aoi022aa1n03x5               g132(.a(new_n227), .b(new_n221), .c(new_n218), .d(new_n224), .o1(\s[20] ));
  nano32aa1n03x5               g133(.a(new_n226), .b(new_n186), .c(new_n221), .d(new_n205), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n196), .c(new_n122), .d(new_n184), .o1(new_n230));
  nanb03aa1n12x5               g135(.a(new_n219), .b(new_n220), .c(new_n214), .out0(new_n231));
  oai112aa1n06x5               g136(.a(new_n225), .b(new_n204), .c(new_n203), .d(new_n210), .o1(new_n232));
  aoi012aa1n12x5               g137(.a(new_n219), .b(new_n213), .c(new_n220), .o1(new_n233));
  oai012aa1n18x5               g138(.a(new_n233), .b(new_n232), .c(new_n231), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  tech160nm_finand02aa1n03p5x5 g140(.a(new_n230), .b(new_n235), .o1(new_n236));
  nor002aa1d32x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nand42aa1n10x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  norb02aa1d27x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  nano22aa1n02x5               g144(.a(new_n219), .b(new_n214), .c(new_n220), .out0(new_n240));
  tech160nm_fioai012aa1n03p5x5 g145(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .o1(new_n241));
  oab012aa1n09x5               g146(.a(new_n241), .b(new_n210), .c(new_n203), .out0(new_n242));
  inv000aa1n02x5               g147(.a(new_n233), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n243), .b(new_n239), .c(new_n242), .d(new_n240), .o1(new_n244));
  aoi022aa1n02x7               g149(.a(new_n236), .b(new_n239), .c(new_n230), .d(new_n244), .o1(\s[21] ));
  nanp02aa1n03x5               g150(.a(new_n236), .b(new_n239), .o1(new_n246));
  nor002aa1n03x5               g151(.a(\b[21] ), .b(\a[22] ), .o1(new_n247));
  nand42aa1n04x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n247), .out0(new_n249));
  aoib12aa1n02x5               g154(.a(new_n237), .b(new_n248), .c(new_n247), .out0(new_n250));
  inv040aa1n08x5               g155(.a(new_n237), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n239), .o1(new_n252));
  aoai13aa1n02x7               g157(.a(new_n251), .b(new_n252), .c(new_n230), .d(new_n235), .o1(new_n253));
  aoi022aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n246), .d(new_n250), .o1(\s[22] ));
  inv000aa1n02x5               g159(.a(new_n229), .o1(new_n255));
  nano22aa1n02x5               g160(.a(new_n255), .b(new_n239), .c(new_n249), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n196), .c(new_n122), .d(new_n184), .o1(new_n257));
  nano23aa1n06x5               g162(.a(new_n237), .b(new_n247), .c(new_n248), .d(new_n238), .out0(new_n258));
  oaoi03aa1n09x5               g163(.a(\a[22] ), .b(\b[21] ), .c(new_n251), .o1(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n234), .c(new_n258), .o1(new_n260));
  tech160nm_finand02aa1n05x5   g165(.a(new_n257), .b(new_n260), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n262), .b(new_n259), .c(new_n234), .d(new_n258), .o1(new_n263));
  aoi022aa1n02x7               g168(.a(new_n261), .b(new_n262), .c(new_n257), .d(new_n263), .o1(\s[23] ));
  nand02aa1n02x5               g169(.a(new_n261), .b(new_n262), .o1(new_n265));
  tech160nm_fixorc02aa1n03p5x5 g170(.a(\a[24] ), .b(\b[23] ), .out0(new_n266));
  nor002aa1d32x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  norp02aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n267), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n262), .o1(new_n270));
  aoai13aa1n02x7               g175(.a(new_n269), .b(new_n270), .c(new_n257), .d(new_n260), .o1(new_n271));
  aoi022aa1n02x7               g176(.a(new_n271), .b(new_n266), .c(new_n265), .d(new_n268), .o1(\s[24] ));
  and002aa1n12x5               g177(.a(new_n266), .b(new_n262), .o(new_n273));
  nano22aa1n02x5               g178(.a(new_n255), .b(new_n273), .c(new_n258), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n196), .c(new_n122), .d(new_n184), .o1(new_n275));
  aoai13aa1n02x7               g180(.a(new_n258), .b(new_n243), .c(new_n242), .d(new_n240), .o1(new_n276));
  inv000aa1n02x5               g181(.a(new_n259), .o1(new_n277));
  inv000aa1n06x5               g182(.a(new_n273), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[24] ), .b(\b[23] ), .c(new_n269), .carry(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n278), .c(new_n276), .d(new_n277), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n280), .o1(new_n281));
  nand42aa1n04x5               g186(.a(new_n275), .b(new_n281), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n273), .b(new_n259), .c(new_n234), .d(new_n258), .o1(new_n284));
  inv000aa1n02x5               g189(.a(new_n283), .o1(new_n285));
  and003aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n279), .o(new_n286));
  aoi022aa1n02x7               g191(.a(new_n282), .b(new_n283), .c(new_n275), .d(new_n286), .o1(\s[25] ));
  nand22aa1n03x5               g192(.a(new_n282), .b(new_n283), .o1(new_n288));
  xorc02aa1n02x5               g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  nor042aa1n06x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n289), .b(new_n290), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n290), .o1(new_n292));
  aoai13aa1n04x5               g197(.a(new_n292), .b(new_n285), .c(new_n275), .d(new_n281), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n293), .b(new_n289), .c(new_n288), .d(new_n291), .o1(\s[26] ));
  inv030aa1d32x5               g199(.a(\a[25] ), .o1(new_n295));
  inv040aa1d32x5               g200(.a(\a[26] ), .o1(new_n296));
  xroi22aa1d06x4               g201(.a(new_n295), .b(\b[24] ), .c(new_n296), .d(\b[25] ), .out0(new_n297));
  nano32aa1n03x7               g202(.a(new_n255), .b(new_n297), .c(new_n258), .d(new_n273), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n196), .c(new_n122), .d(new_n184), .o1(new_n299));
  nor042aa1n03x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  and002aa1n24x5               g205(.a(\b[26] ), .b(\a[27] ), .o(new_n301));
  nor002aa1n03x5               g206(.a(new_n301), .b(new_n300), .o1(new_n302));
  aob012aa1n02x5               g207(.a(new_n290), .b(\b[25] ), .c(\a[26] ), .out0(new_n303));
  oai122aa1n02x7               g208(.a(new_n303), .b(new_n301), .c(new_n300), .d(\a[26] ), .e(\b[25] ), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n280), .c(new_n297), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\b[25] ), .o1(new_n306));
  oaoi03aa1n12x5               g211(.a(new_n296), .b(new_n306), .c(new_n290), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n307), .o1(new_n308));
  tech160nm_fiaoi012aa1n04x5   g213(.a(new_n308), .b(new_n280), .c(new_n297), .o1(new_n309));
  nand42aa1n03x5               g214(.a(new_n309), .b(new_n299), .o1(new_n310));
  aoi022aa1n02x7               g215(.a(new_n310), .b(new_n302), .c(new_n299), .d(new_n305), .o1(\s[27] ));
  inv000aa1d42x5               g216(.a(new_n301), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n297), .o1(new_n313));
  aoai13aa1n04x5               g218(.a(new_n307), .b(new_n313), .c(new_n284), .d(new_n279), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n197), .d(new_n298), .o1(new_n315));
  inv000aa1n03x5               g220(.a(new_n300), .o1(new_n316));
  aoai13aa1n02x7               g221(.a(new_n316), .b(new_n301), .c(new_n309), .d(new_n299), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .out0(new_n318));
  norp02aa1n02x5               g223(.a(new_n318), .b(new_n300), .o1(new_n319));
  aoi022aa1n03x5               g224(.a(new_n317), .b(new_n318), .c(new_n315), .d(new_n319), .o1(\s[28] ));
  and002aa1n03x5               g225(.a(new_n318), .b(new_n302), .o(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n314), .c(new_n197), .d(new_n298), .o1(new_n322));
  inv040aa1n03x5               g227(.a(new_n321), .o1(new_n323));
  oao003aa1n03x5               g228(.a(\a[28] ), .b(\b[27] ), .c(new_n316), .carry(new_n324));
  aoai13aa1n02x7               g229(.a(new_n324), .b(new_n323), .c(new_n309), .d(new_n299), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n324), .b(new_n326), .out0(new_n327));
  aoi022aa1n03x5               g232(.a(new_n325), .b(new_n326), .c(new_n322), .d(new_n327), .o1(\s[29] ));
  nanp02aa1n02x5               g233(.a(\b[0] ), .b(\a[1] ), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n06x5               g235(.a(new_n318), .b(new_n326), .c(new_n302), .o(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n314), .c(new_n197), .d(new_n298), .o1(new_n332));
  inv000aa1n02x5               g237(.a(new_n331), .o1(new_n333));
  inv000aa1d42x5               g238(.a(\b[28] ), .o1(new_n334));
  inv000aa1d42x5               g239(.a(\a[29] ), .o1(new_n335));
  oaib12aa1n02x5               g240(.a(new_n324), .b(\b[28] ), .c(new_n335), .out0(new_n336));
  oaib12aa1n02x5               g241(.a(new_n336), .b(new_n334), .c(\a[29] ), .out0(new_n337));
  aoai13aa1n02x7               g242(.a(new_n337), .b(new_n333), .c(new_n309), .d(new_n299), .o1(new_n338));
  xorc02aa1n02x5               g243(.a(\a[30] ), .b(\b[29] ), .out0(new_n339));
  oaoi13aa1n02x5               g244(.a(new_n339), .b(new_n336), .c(new_n335), .d(new_n334), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n338), .b(new_n339), .c(new_n332), .d(new_n340), .o1(\s[30] ));
  nano22aa1n02x5               g246(.a(new_n323), .b(new_n326), .c(new_n339), .out0(new_n342));
  aoai13aa1n03x5               g247(.a(new_n342), .b(new_n314), .c(new_n197), .d(new_n298), .o1(new_n343));
  aoi022aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n344));
  norb02aa1n02x5               g249(.a(\a[31] ), .b(\b[30] ), .out0(new_n345));
  obai22aa1n02x7               g250(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n346));
  aoi112aa1n02x5               g251(.a(new_n346), .b(new_n345), .c(new_n336), .d(new_n344), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[31] ), .b(\b[30] ), .out0(new_n348));
  inv000aa1n02x5               g253(.a(new_n342), .o1(new_n349));
  norp02aa1n02x5               g254(.a(\b[29] ), .b(\a[30] ), .o1(new_n350));
  aoi012aa1n02x5               g255(.a(new_n350), .b(new_n336), .c(new_n344), .o1(new_n351));
  aoai13aa1n02x7               g256(.a(new_n351), .b(new_n349), .c(new_n309), .d(new_n299), .o1(new_n352));
  aoi022aa1n03x5               g257(.a(new_n352), .b(new_n348), .c(new_n343), .d(new_n347), .o1(\s[31] ));
  nor042aa1n03x5               g258(.a(new_n99), .b(new_n98), .o1(new_n354));
  xnrb03aa1n02x5               g259(.a(new_n354), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  orn002aa1n02x5               g260(.a(\a[3] ), .b(\b[2] ), .o(new_n356));
  oai012aa1n02x5               g261(.a(new_n100), .b(new_n99), .c(new_n98), .o1(new_n357));
  xorc02aa1n02x5               g262(.a(\a[4] ), .b(\b[3] ), .out0(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n358), .b(new_n357), .c(new_n356), .out0(\s[4] ));
  tech160nm_fixnrc02aa1n05x5   g264(.a(\b[2] ), .b(\a[3] ), .out0(new_n360));
  oabi12aa1n06x5               g265(.a(new_n101), .b(new_n354), .c(new_n360), .out0(new_n361));
  xnrc02aa1n02x5               g266(.a(\b[4] ), .b(\a[5] ), .out0(new_n362));
  xnbna2aa1n03x5               g267(.a(new_n362), .b(new_n361), .c(new_n110), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g268(.a(new_n362), .b(new_n361), .c(new_n110), .o(new_n364));
  xorc02aa1n02x5               g269(.a(\a[6] ), .b(\b[5] ), .out0(new_n365));
  xobna2aa1n03x5               g270(.a(new_n365), .b(new_n364), .c(new_n108), .out0(\s[6] ));
  aoai13aa1n06x5               g271(.a(new_n108), .b(new_n362), .c(new_n361), .d(new_n110), .o1(new_n367));
  aoi112aa1n02x5               g272(.a(new_n106), .b(new_n105), .c(\a[6] ), .d(\b[5] ), .o1(new_n368));
  aob012aa1n03x5               g273(.a(new_n368), .b(new_n367), .c(new_n365), .out0(new_n369));
  xnrc02aa1n02x5               g274(.a(\b[6] ), .b(\a[7] ), .out0(new_n370));
  aoai13aa1n02x5               g275(.a(new_n370), .b(new_n117), .c(new_n367), .d(new_n365), .o1(new_n371));
  and002aa1n03x5               g276(.a(new_n371), .b(new_n369), .o(\s[7] ));
  orn002aa1n02x5               g277(.a(\a[7] ), .b(\b[6] ), .o(new_n373));
  xnbna2aa1n03x5               g278(.a(new_n114), .b(new_n369), .c(new_n373), .out0(\s[8] ));
  nanb02aa1n02x5               g279(.a(new_n115), .b(new_n361), .out0(new_n375));
  nanp02aa1n02x5               g280(.a(new_n123), .b(new_n120), .o1(new_n376));
  aoi012aa1n02x5               g281(.a(new_n376), .b(new_n119), .c(new_n116), .o1(new_n377));
  aoi022aa1n02x5               g282(.a(new_n122), .b(new_n124), .c(new_n375), .d(new_n377), .o1(\s[9] ));
endmodule


