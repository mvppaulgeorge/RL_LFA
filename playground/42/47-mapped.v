// Benchmark "adder" written by ABC on Thu Jul 18 09:55:23 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n343, new_n344, new_n346,
    new_n348, new_n349, new_n351, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_finand02aa1n03p5x5 g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor002aa1n16x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  oai022aa1n02x5               g004(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n100));
  xorc02aa1n12x5               g005(.a(\a[3] ), .b(\b[2] ), .out0(new_n101));
  and002aa1n12x5               g006(.a(\b[1] ), .b(\a[2] ), .o(new_n102));
  nand22aa1n12x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n03x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oab012aa1n09x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .out0(new_n105));
  tech160nm_fiaoi012aa1n05x5   g010(.a(new_n100), .b(new_n105), .c(new_n101), .o1(new_n106));
  and002aa1n03x5               g011(.a(\b[7] ), .b(\a[8] ), .o(new_n107));
  nor022aa1n04x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  aoi112aa1n03x5               g013(.a(new_n107), .b(new_n108), .c(\a[4] ), .d(\b[3] ), .o1(new_n109));
  norp02aa1n24x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand22aa1n09x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norb02aa1n06x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  tech160nm_finand02aa1n05x5   g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nano23aa1n03x7               g021(.a(new_n116), .b(new_n113), .c(new_n114), .d(new_n115), .out0(new_n117));
  nand23aa1n03x5               g022(.a(new_n117), .b(new_n109), .c(new_n112), .o1(new_n118));
  aoi022aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n119));
  inv040aa1n02x5               g024(.a(new_n111), .o1(new_n120));
  nor042aa1n06x5               g025(.a(new_n113), .b(new_n110), .o1(new_n121));
  oai022aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(\b[6] ), .d(\a[7] ), .o1(new_n122));
  tech160nm_fiaoi012aa1n04x5   g027(.a(new_n108), .b(new_n122), .c(new_n119), .o1(new_n123));
  oai112aa1n03x5               g028(.a(new_n123), .b(new_n99), .c(new_n118), .d(new_n106), .o1(new_n124));
  nor002aa1n16x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1d28x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n06x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  nor042aa1n09x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n24x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n126), .b(new_n125), .c(new_n124), .d(new_n97), .o1(new_n133));
  tech160nm_fiaoi012aa1n05x5   g038(.a(new_n128), .b(new_n124), .c(new_n97), .o1(new_n134));
  oai012aa1d24x5               g039(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .o1(new_n135));
  aoi112aa1n06x5               g040(.a(new_n134), .b(new_n135), .c(\a[11] ), .d(\b[10] ), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n136), .b(new_n132), .c(new_n133), .o1(\s[11] ));
  nor042aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  tech160nm_fioai012aa1n04x5   g045(.a(new_n140), .b(new_n136), .c(new_n130), .o1(new_n141));
  inv040aa1n03x5               g046(.a(new_n136), .o1(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n130), .out0(new_n143));
  nanp02aa1n03x5               g048(.a(new_n143), .b(new_n141), .o1(\s[12] ));
  oai012aa1n12x5               g049(.a(new_n123), .b(new_n118), .c(new_n106), .o1(new_n145));
  nano23aa1n06x5               g050(.a(new_n140), .b(new_n132), .c(new_n127), .d(new_n99), .out0(new_n146));
  nano22aa1n09x5               g051(.a(new_n138), .b(new_n131), .c(new_n139), .out0(new_n147));
  oab012aa1d15x5               g052(.a(new_n135), .b(new_n98), .c(new_n125), .out0(new_n148));
  tech160nm_fiaoi012aa1n04x5   g053(.a(new_n138), .b(new_n130), .c(new_n139), .o1(new_n149));
  aobi12aa1d24x5               g054(.a(new_n149), .b(new_n148), .c(new_n147), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n151), .c(new_n145), .d(new_n146), .o1(new_n155));
  aoi112aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(new_n145), .d(new_n146), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n152), .o1(new_n158));
  nor042aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand02aa1n10x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n155), .c(new_n158), .out0(\s[14] ));
  nanp02aa1n04x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nor042aa1n09x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  nona22aa1n03x5               g070(.a(new_n155), .b(new_n159), .c(new_n152), .out0(new_n166));
  aoi022aa1n02x7               g071(.a(new_n166), .b(new_n160), .c(new_n163), .d(new_n165), .o1(new_n167));
  nano22aa1n02x4               g072(.a(new_n164), .b(new_n160), .c(new_n163), .out0(new_n168));
  nand02aa1d04x5               g073(.a(new_n166), .b(new_n168), .o1(new_n169));
  norb02aa1n03x4               g074(.a(new_n169), .b(new_n167), .out0(\s[15] ));
  xorc02aa1n12x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n164), .c(new_n166), .d(new_n168), .o1(new_n173));
  nona22aa1n03x5               g078(.a(new_n169), .b(new_n172), .c(new_n164), .out0(new_n174));
  nanp02aa1n03x5               g079(.a(new_n174), .b(new_n173), .o1(\s[16] ));
  oa0022aa1n02x5               g080(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n176));
  aob012aa1n09x5               g081(.a(new_n176), .b(new_n105), .c(new_n101), .out0(new_n177));
  and002aa1n24x5               g082(.a(\b[3] ), .b(\a[4] ), .o(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nor042aa1n02x5               g084(.a(new_n107), .b(new_n108), .o1(new_n180));
  nona23aa1n02x4               g085(.a(new_n115), .b(new_n114), .c(new_n116), .d(new_n113), .out0(new_n181));
  nano32aa1n03x7               g086(.a(new_n181), .b(new_n180), .c(new_n112), .d(new_n179), .out0(new_n182));
  oaoi13aa1n02x5               g087(.a(new_n116), .b(new_n111), .c(new_n110), .d(new_n113), .o1(new_n183));
  obai22aa1n03x5               g088(.a(new_n119), .b(new_n183), .c(\a[8] ), .d(\b[7] ), .out0(new_n184));
  nano23aa1n09x5               g089(.a(new_n138), .b(new_n98), .c(new_n139), .d(new_n97), .out0(new_n185));
  nano23aa1n03x7               g090(.a(new_n125), .b(new_n130), .c(new_n131), .d(new_n126), .out0(new_n186));
  norb02aa1n09x5               g091(.a(new_n163), .b(new_n164), .out0(new_n187));
  nano23aa1n06x5               g092(.a(new_n152), .b(new_n159), .c(new_n160), .d(new_n153), .out0(new_n188));
  nand23aa1d12x5               g093(.a(new_n188), .b(new_n187), .c(new_n171), .o1(new_n189));
  nano22aa1d15x5               g094(.a(new_n189), .b(new_n185), .c(new_n186), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n184), .c(new_n177), .d(new_n182), .o1(new_n191));
  nor042aa1n06x5               g096(.a(new_n150), .b(new_n189), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\a[16] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[15] ), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n163), .b(new_n159), .c(new_n152), .d(new_n160), .o1(new_n195));
  nand42aa1n04x5               g100(.a(new_n195), .b(new_n165), .o1(new_n196));
  oaoi03aa1n12x5               g101(.a(new_n193), .b(new_n194), .c(new_n196), .o1(new_n197));
  norb02aa1d21x5               g102(.a(new_n197), .b(new_n192), .out0(new_n198));
  xorc02aa1n12x5               g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n191), .c(new_n198), .out0(\s[17] ));
  inv000aa1d42x5               g105(.a(\a[18] ), .o1(new_n201));
  nanp02aa1n06x5               g106(.a(new_n191), .b(new_n198), .o1(new_n202));
  nor042aa1d18x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  tech160nm_fiaoi012aa1n05x5   g108(.a(new_n203), .b(new_n202), .c(new_n199), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[17] ), .c(new_n201), .out0(\s[18] ));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nand02aa1n20x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nor002aa1n20x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n199), .o1(new_n210));
  nor042aa1d18x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nor002aa1n03x5               g116(.a(new_n211), .b(new_n203), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n210), .c(new_n191), .d(new_n198), .o1(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n209), .b(new_n213), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nano22aa1n02x4               g120(.a(new_n208), .b(new_n206), .c(new_n207), .out0(new_n216));
  nor002aa1d32x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand42aa1d28x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n208), .c(new_n213), .d(new_n216), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(new_n213), .b(new_n216), .o1(new_n221));
  nona22aa1n02x5               g126(.a(new_n221), .b(new_n219), .c(new_n208), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n222), .b(new_n220), .o1(\s[20] ));
  nanb03aa1n12x5               g128(.a(new_n217), .b(new_n218), .c(new_n207), .out0(new_n224));
  orn002aa1n24x5               g129(.a(\a[19] ), .b(\b[18] ), .o(new_n225));
  oai112aa1n06x5               g130(.a(new_n225), .b(new_n206), .c(new_n211), .d(new_n203), .o1(new_n226));
  oai012aa1d24x5               g131(.a(new_n218), .b(new_n217), .c(new_n208), .o1(new_n227));
  oai012aa1n18x5               g132(.a(new_n227), .b(new_n226), .c(new_n224), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  xnrc02aa1n02x5               g134(.a(\b[17] ), .b(\a[18] ), .out0(new_n230));
  nano23aa1n06x5               g135(.a(new_n217), .b(new_n208), .c(new_n218), .d(new_n207), .out0(new_n231));
  nanb03aa1n12x5               g136(.a(new_n230), .b(new_n231), .c(new_n199), .out0(new_n232));
  aoai13aa1n04x5               g137(.a(new_n229), .b(new_n232), .c(new_n191), .d(new_n198), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n232), .c(new_n191), .d(new_n198), .o1(new_n236));
  aboi22aa1n03x5               g141(.a(new_n236), .b(new_n229), .c(new_n233), .d(new_n234), .out0(\s[21] ));
  nor002aa1d32x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[21] ), .b(\a[22] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n238), .c(new_n233), .d(new_n234), .o1(new_n240));
  nand02aa1n02x5               g145(.a(new_n233), .b(new_n234), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n241), .b(new_n239), .c(new_n238), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n242), .b(new_n240), .o1(\s[22] ));
  nor042aa1n02x5               g148(.a(new_n239), .b(new_n235), .o1(new_n244));
  nona23aa1n02x4               g149(.a(new_n244), .b(new_n231), .c(new_n210), .d(new_n230), .out0(new_n245));
  nano22aa1d15x5               g150(.a(new_n217), .b(new_n207), .c(new_n218), .out0(new_n246));
  tech160nm_fioai012aa1n04x5   g151(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n247));
  nona22aa1n09x5               g152(.a(new_n246), .b(new_n212), .c(new_n247), .out0(new_n248));
  nanb02aa1n12x5               g153(.a(new_n239), .b(new_n234), .out0(new_n249));
  inv040aa1d28x5               g154(.a(\a[22] ), .o1(new_n250));
  inv040aa1d32x5               g155(.a(\b[21] ), .o1(new_n251));
  oaoi03aa1n12x5               g156(.a(new_n250), .b(new_n251), .c(new_n238), .o1(new_n252));
  aoai13aa1n12x5               g157(.a(new_n252), .b(new_n249), .c(new_n248), .d(new_n227), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n245), .c(new_n191), .d(new_n198), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  and002aa1n24x5               g162(.a(\b[22] ), .b(\a[23] ), .o(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n255), .d(new_n259), .o1(new_n261));
  nor042aa1n06x5               g166(.a(new_n258), .b(new_n257), .o1(new_n262));
  nand02aa1n02x5               g167(.a(new_n255), .b(new_n262), .o1(new_n263));
  nona22aa1n02x5               g168(.a(new_n263), .b(new_n260), .c(new_n257), .out0(new_n264));
  nanp02aa1n03x5               g169(.a(new_n264), .b(new_n261), .o1(\s[24] ));
  tech160nm_fioai012aa1n03p5x5 g170(.a(new_n197), .b(new_n189), .c(new_n150), .o1(new_n266));
  norb02aa1n02x7               g171(.a(new_n262), .b(new_n260), .out0(new_n267));
  nano22aa1n02x5               g172(.a(new_n232), .b(new_n267), .c(new_n244), .out0(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n266), .c(new_n145), .d(new_n190), .o1(new_n269));
  nand02aa1n06x5               g174(.a(\b[23] ), .b(\a[24] ), .o1(new_n270));
  oab012aa1n09x5               g175(.a(new_n257), .b(\a[24] ), .c(\b[23] ), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n270), .b(new_n272), .c(new_n253), .d(new_n259), .o1(new_n273));
  nor002aa1d32x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  and002aa1n02x5               g179(.a(\b[24] ), .b(\a[25] ), .o(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n274), .o1(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n269), .c(new_n273), .out0(\s[25] ));
  aob012aa1n03x5               g182(.a(new_n276), .b(new_n269), .c(new_n273), .out0(new_n278));
  inv040aa1n03x5               g183(.a(new_n274), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n275), .c(new_n269), .d(new_n273), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n274), .o1(new_n282));
  aoi022aa1n03x5               g187(.a(new_n280), .b(new_n281), .c(new_n278), .d(new_n282), .o1(\s[26] ));
  nand42aa1n03x5               g188(.a(\b[25] ), .b(\a[26] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\a[26] ), .o1(new_n285));
  inv040aa1d32x5               g190(.a(\b[25] ), .o1(new_n286));
  ao0022aa1n12x5               g191(.a(new_n285), .b(new_n286), .c(\a[25] ), .d(\b[24] ), .o(new_n287));
  nano32aa1d15x5               g192(.a(new_n287), .b(new_n284), .c(new_n279), .d(new_n270), .out0(new_n288));
  nano32aa1n03x7               g193(.a(new_n232), .b(new_n288), .c(new_n244), .d(new_n267), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n266), .c(new_n145), .d(new_n190), .o1(new_n290));
  aoai13aa1n04x5               g195(.a(new_n288), .b(new_n272), .c(new_n253), .d(new_n259), .o1(new_n291));
  oaoi03aa1n02x5               g196(.a(new_n285), .b(new_n286), .c(new_n274), .o1(new_n292));
  nand23aa1n03x5               g197(.a(new_n290), .b(new_n291), .c(new_n292), .o1(new_n293));
  tech160nm_fixorc02aa1n03p5x5 g198(.a(\a[27] ), .b(\b[26] ), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  aoi022aa1n02x5               g200(.a(new_n295), .b(new_n290), .c(new_n293), .d(new_n294), .o1(\s[27] ));
  norp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  norp02aa1n02x5               g202(.a(\b[27] ), .b(\a[28] ), .o1(new_n298));
  nand42aa1n03x5               g203(.a(\b[27] ), .b(\a[28] ), .o1(new_n299));
  nanb02aa1n06x5               g204(.a(new_n298), .b(new_n299), .out0(new_n300));
  aoai13aa1n02x7               g205(.a(new_n300), .b(new_n297), .c(new_n293), .d(new_n294), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n252), .o1(new_n302));
  aoai13aa1n06x5               g207(.a(new_n259), .b(new_n302), .c(new_n228), .d(new_n244), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n288), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n292), .b(new_n304), .c(new_n303), .d(new_n271), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n294), .b(new_n305), .c(new_n202), .d(new_n289), .o1(new_n306));
  nona22aa1n02x5               g211(.a(new_n306), .b(new_n300), .c(new_n297), .out0(new_n307));
  nanp02aa1n03x5               g212(.a(new_n301), .b(new_n307), .o1(\s[28] ));
  norb02aa1n03x5               g213(.a(new_n294), .b(new_n300), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n202), .d(new_n289), .o1(new_n310));
  oab012aa1n02x4               g215(.a(new_n247), .b(new_n203), .c(new_n211), .out0(new_n311));
  inv000aa1d42x5               g216(.a(new_n227), .o1(new_n312));
  aoai13aa1n02x5               g217(.a(new_n244), .b(new_n312), .c(new_n311), .d(new_n246), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n271), .b(new_n258), .c(new_n313), .d(new_n252), .o1(new_n314));
  aobi12aa1n02x5               g219(.a(new_n292), .b(new_n314), .c(new_n288), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n309), .o1(new_n316));
  oai012aa1n02x5               g221(.a(new_n299), .b(new_n298), .c(new_n297), .o1(new_n317));
  aoai13aa1n02x7               g222(.a(new_n317), .b(new_n316), .c(new_n315), .d(new_n290), .o1(new_n318));
  norp02aa1n02x5               g223(.a(\b[28] ), .b(\a[29] ), .o1(new_n319));
  nand42aa1n03x5               g224(.a(\b[28] ), .b(\a[29] ), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(new_n321));
  oai022aa1n02x5               g226(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n322));
  aboi22aa1n03x5               g227(.a(new_n319), .b(new_n320), .c(new_n322), .d(new_n299), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n318), .b(new_n321), .c(new_n310), .d(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g230(.a(new_n300), .b(new_n294), .c(new_n321), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n305), .c(new_n202), .d(new_n289), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n326), .o1(new_n328));
  aoi013aa1n02x4               g233(.a(new_n319), .b(new_n322), .c(new_n299), .d(new_n320), .o1(new_n329));
  aoai13aa1n02x7               g234(.a(new_n329), .b(new_n328), .c(new_n315), .d(new_n290), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .out0(new_n331));
  aoi113aa1n02x5               g236(.a(new_n331), .b(new_n319), .c(new_n322), .d(new_n320), .e(new_n299), .o1(new_n332));
  aoi022aa1n03x5               g237(.a(new_n330), .b(new_n331), .c(new_n327), .d(new_n332), .o1(\s[30] ));
  nand23aa1n03x5               g238(.a(new_n309), .b(new_n321), .c(new_n331), .o1(new_n334));
  nanb02aa1n03x5               g239(.a(new_n334), .b(new_n293), .out0(new_n335));
  xorc02aa1n02x5               g240(.a(\a[31] ), .b(\b[30] ), .out0(new_n336));
  oao003aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(new_n329), .carry(new_n337));
  norb02aa1n02x5               g242(.a(new_n337), .b(new_n336), .out0(new_n338));
  aoai13aa1n02x5               g243(.a(new_n337), .b(new_n334), .c(new_n315), .d(new_n290), .o1(new_n339));
  aoi022aa1n03x5               g244(.a(new_n339), .b(new_n336), .c(new_n335), .d(new_n338), .o1(\s[31] ));
  xorb03aa1n02x5               g245(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  orn002aa1n02x5               g246(.a(\a[4] ), .b(\b[3] ), .o(new_n342));
  obai22aa1n02x7               g247(.a(new_n342), .b(new_n178), .c(\a[3] ), .d(\b[2] ), .out0(new_n343));
  aoi012aa1n02x5               g248(.a(new_n343), .b(new_n101), .c(new_n105), .o1(new_n344));
  aoi013aa1n02x4               g249(.a(new_n344), .b(new_n177), .c(new_n179), .d(new_n342), .o1(\s[4] ));
  aoai13aa1n02x5               g250(.a(new_n179), .b(new_n100), .c(new_n105), .d(new_n101), .o1(new_n346));
  xnrb03aa1n02x5               g251(.a(new_n346), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g252(.a(new_n113), .o1(new_n348));
  nona23aa1n02x4               g253(.a(new_n177), .b(new_n114), .c(new_n113), .d(new_n178), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n112), .b(new_n349), .c(new_n348), .out0(\s[6] ));
  norb02aa1n02x5               g255(.a(new_n115), .b(new_n116), .out0(new_n351));
  nona32aa1n02x4               g256(.a(new_n349), .b(new_n113), .c(new_n120), .d(new_n110), .out0(new_n352));
  xobna2aa1n03x5               g257(.a(new_n351), .b(new_n352), .c(new_n111), .out0(\s[7] ));
  aoi013aa1n02x4               g258(.a(new_n116), .b(new_n352), .c(new_n111), .d(new_n115), .o1(new_n354));
  xnrc02aa1n02x5               g259(.a(new_n354), .b(new_n180), .out0(\s[8] ));
  xorb03aa1n02x5               g260(.a(new_n145), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


