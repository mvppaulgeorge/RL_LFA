// Benchmark "adder" written by ABC on Wed Jul 17 15:33:49 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n342, new_n343, new_n345,
    new_n346, new_n347, new_n349, new_n350, new_n351, new_n353, new_n354,
    new_n355, new_n357, new_n358, new_n359, new_n360, new_n362, new_n363;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n04x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1d04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1n06x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nor022aa1n04x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand22aa1n04x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nona23aa1n06x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  nor043aa1d12x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  tech160nm_fiaoi012aa1n05x5   g022(.a(new_n112), .b(new_n110), .c(new_n113), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[8] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[7] ), .o1(new_n120));
  norp02aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  oai013aa1n09x5               g027(.a(new_n122), .b(new_n116), .c(new_n115), .d(new_n118), .o1(new_n123));
  nand42aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n126));
  nor042aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nano23aa1n02x5               g035(.a(new_n97), .b(new_n127), .c(new_n128), .d(new_n124), .out0(new_n131));
  aoai13aa1n03x5               g036(.a(new_n131), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n132));
  aoi012aa1n03x5               g037(.a(new_n127), .b(new_n97), .c(new_n128), .o1(new_n133));
  norp02aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n03x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aobi12aa1n06x5               g041(.a(new_n136), .b(new_n132), .c(new_n133), .out0(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n127), .c(new_n128), .d(new_n97), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n137), .b(new_n132), .c(new_n138), .o1(\s[11] ));
  norp02aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  oabi12aa1n02x7               g047(.a(new_n142), .b(new_n137), .c(new_n134), .out0(new_n143));
  norb03aa1n02x5               g048(.a(new_n141), .b(new_n134), .c(new_n140), .out0(new_n144));
  oaib12aa1n02x5               g049(.a(new_n143), .b(new_n137), .c(new_n144), .out0(\s[12] ));
  nona23aa1n03x5               g050(.a(new_n141), .b(new_n135), .c(new_n134), .d(new_n140), .out0(new_n146));
  nano22aa1n02x4               g051(.a(new_n146), .b(new_n125), .c(new_n129), .out0(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n148));
  nano23aa1n06x5               g053(.a(new_n134), .b(new_n140), .c(new_n141), .d(new_n135), .out0(new_n149));
  tech160nm_fiao0012aa1n05x5   g054(.a(new_n140), .b(new_n134), .c(new_n141), .o(new_n150));
  aoib12aa1n06x5               g055(.a(new_n150), .b(new_n149), .c(new_n133), .out0(new_n151));
  nor042aa1d18x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1d28x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(new_n152), .o1(new_n156));
  nanp02aa1n03x5               g061(.a(new_n148), .b(new_n151), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(new_n157), .b(new_n154), .o1(new_n158));
  nor042aa1n09x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1d28x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n158), .c(new_n156), .out0(\s[14] ));
  nano23aa1d15x5               g067(.a(new_n152), .b(new_n159), .c(new_n160), .d(new_n153), .out0(new_n163));
  nanp02aa1n02x5               g068(.a(new_n157), .b(new_n163), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n163), .o1(new_n165));
  tech160nm_fiaoi012aa1n04x5   g070(.a(new_n159), .b(new_n152), .c(new_n160), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n148), .d(new_n151), .o1(new_n167));
  xorc02aa1n12x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n159), .c(new_n160), .d(new_n152), .o1(new_n169));
  aoi022aa1n02x5               g074(.a(new_n167), .b(new_n168), .c(new_n164), .d(new_n169), .o1(\s[15] ));
  norp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nor042aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand02aa1d16x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n06x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  inv030aa1n03x5               g079(.a(new_n174), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n171), .c(new_n167), .d(new_n168), .o1(new_n176));
  xnrc02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .out0(new_n177));
  norb03aa1n02x5               g082(.a(new_n173), .b(new_n171), .c(new_n172), .out0(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n177), .c(new_n164), .d(new_n166), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n176), .b(new_n179), .o1(\s[16] ));
  nand23aa1d12x5               g085(.a(new_n163), .b(new_n168), .c(new_n174), .o1(new_n181));
  nano22aa1d15x5               g086(.a(new_n181), .b(new_n131), .c(new_n149), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n183));
  oabi12aa1n03x5               g088(.a(new_n150), .b(new_n146), .c(new_n133), .out0(new_n184));
  aoi012aa1n02x5               g089(.a(new_n172), .b(new_n171), .c(new_n173), .o1(new_n185));
  oai013aa1n03x4               g090(.a(new_n185), .b(new_n175), .c(new_n166), .d(new_n177), .o1(new_n186));
  aoib12aa1n09x5               g091(.a(new_n186), .b(new_n184), .c(new_n181), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n183), .b(new_n187), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  inv000aa1d42x5               g094(.a(new_n181), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n186), .b(new_n189), .c(new_n190), .d(new_n184), .o1(new_n191));
  aoi022aa1n02x5               g096(.a(new_n188), .b(new_n189), .c(new_n183), .d(new_n191), .o1(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[17] ), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(\b[16] ), .b(new_n193), .out0(new_n194));
  nand22aa1n04x5               g099(.a(new_n109), .b(new_n117), .o1(new_n195));
  nor003aa1n02x5               g100(.a(new_n115), .b(new_n116), .c(new_n118), .o1(new_n196));
  norb02aa1n03x5               g101(.a(new_n122), .b(new_n196), .out0(new_n197));
  nanp02aa1n12x5               g102(.a(new_n195), .b(new_n197), .o1(new_n198));
  oabi12aa1n09x5               g103(.a(new_n186), .b(new_n151), .c(new_n181), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n189), .b(new_n199), .c(new_n198), .d(new_n182), .o1(new_n200));
  xorc02aa1n12x5               g105(.a(\a[18] ), .b(\b[17] ), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n200), .c(new_n194), .out0(\s[18] ));
  inv040aa1d32x5               g107(.a(\a[18] ), .o1(new_n203));
  xroi22aa1d04x5               g108(.a(new_n193), .b(\b[16] ), .c(new_n203), .d(\b[17] ), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n204), .b(new_n199), .c(new_n198), .d(new_n182), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n204), .o1(new_n206));
  nor042aa1n06x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  inv040aa1d32x5               g112(.a(\b[17] ), .o1(new_n208));
  nand42aa1n16x5               g113(.a(new_n208), .b(new_n203), .o1(new_n209));
  nand02aa1n06x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  aob012aa1d15x5               g115(.a(new_n209), .b(new_n207), .c(new_n210), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n04x5               g117(.a(new_n212), .b(new_n206), .c(new_n183), .d(new_n187), .o1(new_n213));
  nor022aa1n08x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand42aa1n06x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1n15x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n209), .o1(new_n217));
  aoi112aa1n02x5               g122(.a(new_n216), .b(new_n217), .c(new_n210), .d(new_n207), .o1(new_n218));
  aoi022aa1n02x5               g123(.a(new_n213), .b(new_n216), .c(new_n205), .d(new_n218), .o1(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nanp02aa1n09x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n09x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n214), .c(new_n213), .d(new_n216), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n216), .o1(new_n226));
  norb03aa1n02x5               g131(.a(new_n222), .b(new_n214), .c(new_n221), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n226), .c(new_n205), .d(new_n212), .o1(new_n228));
  nanp02aa1n03x5               g133(.a(new_n225), .b(new_n228), .o1(\s[20] ));
  nano23aa1n06x5               g134(.a(new_n214), .b(new_n221), .c(new_n222), .d(new_n215), .out0(new_n230));
  nand23aa1n06x5               g135(.a(new_n230), .b(new_n189), .c(new_n201), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n199), .c(new_n198), .d(new_n182), .o1(new_n233));
  aoi012aa1n02x7               g138(.a(new_n221), .b(new_n214), .c(new_n222), .o1(new_n234));
  aobi12aa1n02x7               g139(.a(new_n234), .b(new_n230), .c(new_n211), .out0(new_n235));
  nor002aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nand02aa1d24x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n03x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n233), .c(new_n235), .out0(\s[21] ));
  aoai13aa1n03x5               g144(.a(new_n235), .b(new_n231), .c(new_n183), .d(new_n187), .o1(new_n240));
  nor002aa1n10x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nand02aa1d28x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  norb02aa1n03x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n236), .c(new_n240), .d(new_n238), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n238), .o1(new_n246));
  norb03aa1n02x5               g151(.a(new_n242), .b(new_n236), .c(new_n241), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n246), .c(new_n233), .d(new_n235), .o1(new_n248));
  nanp02aa1n03x5               g153(.a(new_n245), .b(new_n248), .o1(\s[22] ));
  nano23aa1d12x5               g154(.a(new_n236), .b(new_n241), .c(new_n242), .d(new_n237), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  nano32aa1n02x4               g156(.a(new_n251), .b(new_n230), .c(new_n201), .d(new_n189), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n199), .c(new_n198), .d(new_n182), .o1(new_n253));
  inv000aa1n02x5               g158(.a(new_n252), .o1(new_n254));
  tech160nm_fiao0012aa1n02p5x5 g159(.a(new_n241), .b(new_n236), .c(new_n242), .o(new_n255));
  oab012aa1n02x4               g160(.a(new_n255), .b(new_n235), .c(new_n251), .out0(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n254), .c(new_n183), .d(new_n187), .o1(new_n257));
  nor042aa1n06x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  nand02aa1n03x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  nanb02aa1n02x5               g164(.a(new_n258), .b(new_n259), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n261), .b(new_n241), .c(new_n243), .d(new_n236), .o1(new_n262));
  oa0012aa1n02x5               g167(.a(new_n262), .b(new_n235), .c(new_n251), .o(new_n263));
  aoi022aa1n02x5               g168(.a(new_n257), .b(new_n261), .c(new_n253), .d(new_n263), .o1(\s[23] ));
  nor042aa1n04x5               g169(.a(\b[23] ), .b(\a[24] ), .o1(new_n265));
  nand02aa1d06x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  nanb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n258), .c(new_n257), .d(new_n261), .o1(new_n268));
  norb03aa1n02x5               g173(.a(new_n266), .b(new_n258), .c(new_n265), .out0(new_n269));
  aoai13aa1n02x7               g174(.a(new_n269), .b(new_n260), .c(new_n253), .d(new_n256), .o1(new_n270));
  nanp02aa1n03x5               g175(.a(new_n268), .b(new_n270), .o1(\s[24] ));
  nano23aa1n03x7               g176(.a(new_n258), .b(new_n265), .c(new_n266), .d(new_n259), .out0(new_n272));
  nand02aa1d04x5               g177(.a(new_n272), .b(new_n250), .o1(new_n273));
  nano32aa1n02x4               g178(.a(new_n273), .b(new_n230), .c(new_n201), .d(new_n189), .out0(new_n274));
  aoai13aa1n02x7               g179(.a(new_n274), .b(new_n199), .c(new_n198), .d(new_n182), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n274), .o1(new_n276));
  nanp03aa1n06x5               g181(.a(new_n211), .b(new_n216), .c(new_n223), .o1(new_n277));
  tech160nm_fiao0012aa1n02p5x5 g182(.a(new_n265), .b(new_n258), .c(new_n266), .o(new_n278));
  aoi012aa1n06x5               g183(.a(new_n278), .b(new_n272), .c(new_n255), .o1(new_n279));
  aoai13aa1n12x5               g184(.a(new_n279), .b(new_n273), .c(new_n277), .d(new_n234), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n281), .b(new_n276), .c(new_n183), .d(new_n187), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  oai112aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n235), .d(new_n273), .o1(new_n285));
  aboi22aa1n03x5               g190(.a(new_n285), .b(new_n275), .c(new_n282), .d(new_n283), .out0(\s[25] ));
  norp02aa1n02x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[25] ), .b(\a[26] ), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n287), .c(new_n282), .d(new_n283), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n288), .b(new_n287), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n284), .c(new_n275), .d(new_n281), .o1(new_n291));
  nanp02aa1n03x5               g196(.a(new_n289), .b(new_n291), .o1(\s[26] ));
  norb02aa1n02x5               g197(.a(new_n283), .b(new_n288), .out0(new_n293));
  norb03aa1n03x5               g198(.a(new_n293), .b(new_n231), .c(new_n273), .out0(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n199), .c(new_n198), .d(new_n182), .o1(new_n295));
  inv000aa1n02x5               g200(.a(new_n294), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\a[26] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[25] ), .o1(new_n298));
  oao003aa1n02x5               g203(.a(new_n297), .b(new_n298), .c(new_n287), .carry(new_n299));
  aoi012aa1n12x5               g204(.a(new_n299), .b(new_n280), .c(new_n293), .o1(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n296), .c(new_n183), .d(new_n187), .o1(new_n301));
  xorc02aa1n12x5               g206(.a(\a[27] ), .b(\b[26] ), .out0(new_n302));
  nanb02aa1n02x5               g207(.a(new_n288), .b(new_n287), .out0(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  oai112aa1n02x5               g209(.a(new_n303), .b(new_n304), .c(\b[25] ), .d(\a[26] ), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n305), .b(new_n280), .c(new_n293), .o1(new_n306));
  aoi022aa1n02x5               g211(.a(new_n301), .b(new_n302), .c(new_n295), .d(new_n306), .o1(\s[27] ));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  xnrc02aa1n12x5               g213(.a(\b[27] ), .b(\a[28] ), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n301), .d(new_n302), .o1(new_n310));
  nor042aa1n03x5               g215(.a(new_n309), .b(new_n308), .o1(new_n311));
  aoai13aa1n02x5               g216(.a(new_n311), .b(new_n304), .c(new_n295), .d(new_n300), .o1(new_n312));
  nanp02aa1n03x5               g217(.a(new_n310), .b(new_n312), .o1(\s[28] ));
  norb02aa1n09x5               g218(.a(new_n302), .b(new_n309), .out0(new_n314));
  tech160nm_fioaoi03aa1n03p5x5 g219(.a(\a[28] ), .b(\b[27] ), .c(new_n311), .o1(new_n315));
  xorc02aa1n12x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n315), .c(new_n301), .d(new_n314), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  norp02aa1n02x5               g224(.a(new_n315), .b(new_n317), .o1(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n319), .c(new_n295), .d(new_n300), .o1(new_n321));
  nanp02aa1n03x5               g226(.a(new_n318), .b(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g227(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g228(.a(new_n309), .b(new_n302), .c(new_n316), .out0(new_n324));
  nanp02aa1n03x5               g229(.a(new_n301), .b(new_n324), .o1(new_n325));
  norp02aa1n02x5               g230(.a(\b[28] ), .b(\a[29] ), .o1(new_n326));
  tech160nm_fiaoi012aa1n05x5   g231(.a(new_n326), .b(new_n315), .c(new_n316), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  inv000aa1n02x5               g233(.a(new_n324), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n328), .o1(new_n330));
  aoi112aa1n02x5               g235(.a(new_n326), .b(new_n330), .c(new_n315), .d(new_n316), .o1(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n329), .c(new_n295), .d(new_n300), .o1(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n328), .c(new_n325), .d(new_n327), .o1(\s[30] ));
  nano23aa1n06x5               g238(.a(new_n330), .b(new_n309), .c(new_n302), .d(new_n316), .out0(new_n334));
  oaoi03aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .c(new_n327), .o1(new_n335));
  xnrc02aa1n02x5               g240(.a(\b[30] ), .b(\a[31] ), .out0(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n301), .d(new_n334), .o1(new_n337));
  inv000aa1n02x5               g242(.a(new_n334), .o1(new_n338));
  norp02aa1n02x5               g243(.a(new_n335), .b(new_n336), .o1(new_n339));
  aoai13aa1n02x5               g244(.a(new_n339), .b(new_n338), .c(new_n295), .d(new_n300), .o1(new_n340));
  nanp02aa1n03x5               g245(.a(new_n337), .b(new_n340), .o1(\s[31] ));
  norb02aa1n02x5               g246(.a(new_n106), .b(new_n105), .out0(new_n342));
  aoi112aa1n02x5               g247(.a(new_n342), .b(new_n99), .c(new_n100), .d(new_n101), .o1(new_n343));
  aoib12aa1n02x5               g248(.a(new_n343), .b(new_n342), .c(new_n102), .out0(\s[3] ));
  norb02aa1n02x5               g249(.a(new_n104), .b(new_n103), .out0(new_n345));
  inv000aa1d42x5               g250(.a(new_n105), .o1(new_n346));
  aoai13aa1n02x5               g251(.a(new_n342), .b(new_n99), .c(new_n101), .d(new_n100), .o1(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n345), .b(new_n347), .c(new_n346), .out0(\s[4] ));
  nanb03aa1n02x5               g253(.a(new_n102), .b(new_n342), .c(new_n345), .out0(new_n349));
  norb02aa1n02x5               g254(.a(new_n111), .b(new_n110), .out0(new_n350));
  aoi112aa1n02x5               g255(.a(new_n350), .b(new_n103), .c(new_n104), .d(new_n105), .o1(new_n351));
  aoi022aa1n02x5               g256(.a(new_n109), .b(new_n350), .c(new_n349), .d(new_n351), .o1(\s[5] ));
  nanb02aa1n02x5               g257(.a(new_n112), .b(new_n113), .out0(new_n353));
  aoai13aa1n02x5               g258(.a(new_n353), .b(new_n110), .c(new_n109), .d(new_n111), .o1(new_n354));
  nona22aa1n02x4               g259(.a(new_n113), .b(new_n112), .c(new_n110), .out0(new_n355));
  aoai13aa1n02x5               g260(.a(new_n354), .b(new_n355), .c(new_n350), .d(new_n109), .o1(\s[6] ));
  inv000aa1d42x5               g261(.a(new_n116), .o1(new_n357));
  nanb02aa1n02x5               g262(.a(new_n114), .b(new_n109), .out0(new_n358));
  aoai13aa1n02x5               g263(.a(new_n118), .b(new_n114), .c(new_n349), .d(new_n108), .o1(new_n359));
  aoi112aa1n02x5               g264(.a(new_n357), .b(new_n112), .c(new_n113), .d(new_n110), .o1(new_n360));
  aoi022aa1n02x5               g265(.a(new_n359), .b(new_n357), .c(new_n358), .d(new_n360), .o1(\s[7] ));
  oabi12aa1n02x5               g266(.a(new_n115), .b(\a[7] ), .c(\b[6] ), .out0(new_n362));
  aoai13aa1n02x5               g267(.a(new_n115), .b(new_n121), .c(new_n359), .d(new_n357), .o1(new_n363));
  aoai13aa1n02x5               g268(.a(new_n363), .b(new_n362), .c(new_n357), .d(new_n359), .o1(\s[8] ));
  xnbna2aa1n03x5               g269(.a(new_n125), .b(new_n195), .c(new_n197), .out0(\s[9] ));
endmodule


