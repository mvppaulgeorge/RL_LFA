// Benchmark "adder" written by ABC on Thu Jul 18 02:39:24 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n351,
    new_n353, new_n354, new_n356, new_n357, new_n358, new_n359, new_n360,
    new_n362, new_n363, new_n364, new_n367, new_n368, new_n370, new_n371;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d24x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  norb03aa1n09x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  tech160nm_fioai012aa1n03p5x5 g007(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .o1(new_n103));
  nand02aa1d16x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand42aa1n08x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nanb03aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n107));
  nor002aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  aoi012aa1n03x5               g013(.a(new_n106), .b(new_n108), .c(new_n104), .o1(new_n109));
  oai013aa1n03x5               g014(.a(new_n109), .b(new_n102), .c(new_n107), .d(new_n103), .o1(new_n110));
  nor022aa1n04x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nand22aa1n12x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanp02aa1n09x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nona23aa1d18x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[7] ), .b(\b[6] ), .out0(new_n116));
  nand42aa1n20x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor002aa1n06x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norb02aa1n03x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nano22aa1n03x7               g024(.a(new_n115), .b(new_n116), .c(new_n119), .out0(new_n120));
  nano22aa1n03x7               g025(.a(new_n118), .b(new_n113), .c(new_n117), .out0(new_n121));
  oai022aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nanp03aa1n02x5               g027(.a(new_n121), .b(new_n116), .c(new_n122), .o1(new_n123));
  nor042aa1n04x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  aoi012aa1d18x5               g029(.a(new_n118), .b(new_n124), .c(new_n117), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n123), .b(new_n125), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n128));
  nor002aa1d32x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1d28x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanb02aa1d36x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  inv040aa1d32x5               g038(.a(\a[11] ), .o1(new_n134));
  inv040aa1d32x5               g039(.a(\b[10] ), .o1(new_n135));
  nand02aa1d16x5               g040(.a(new_n135), .b(new_n134), .o1(new_n136));
  nand02aa1n08x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanp02aa1n03x5               g042(.a(new_n136), .b(new_n137), .o1(new_n138));
  norp02aa1n02x5               g043(.a(new_n129), .b(new_n97), .o1(new_n139));
  aob012aa1n02x5               g044(.a(new_n130), .b(new_n128), .c(new_n139), .out0(new_n140));
  nanp03aa1n02x5               g045(.a(new_n136), .b(new_n130), .c(new_n137), .o1(new_n141));
  ao0012aa1n03x7               g046(.a(new_n141), .b(new_n128), .c(new_n139), .o(new_n142));
  aobi12aa1n02x5               g047(.a(new_n142), .b(new_n140), .c(new_n138), .out0(\s[11] ));
  aoai13aa1n02x5               g048(.a(new_n136), .b(new_n141), .c(new_n128), .d(new_n139), .o1(new_n144));
  nor042aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand42aa1n08x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n06x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  aboi22aa1n03x5               g052(.a(new_n145), .b(new_n146), .c(new_n134), .d(new_n135), .out0(new_n148));
  aoi022aa1n02x5               g053(.a(new_n142), .b(new_n148), .c(new_n144), .d(new_n147), .o1(\s[12] ));
  nona23aa1d16x5               g054(.a(new_n147), .b(new_n127), .c(new_n131), .d(new_n138), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n152));
  nano22aa1n06x5               g057(.a(new_n145), .b(new_n137), .c(new_n146), .out0(new_n153));
  oai012aa1n02x5               g058(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .o1(new_n154));
  oab012aa1n04x5               g059(.a(new_n154), .b(new_n97), .c(new_n129), .out0(new_n155));
  oaoi03aa1n12x5               g060(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aob012aa1n02x5               g062(.a(new_n157), .b(new_n155), .c(new_n153), .out0(new_n158));
  nanb02aa1n02x5               g063(.a(new_n158), .b(new_n152), .out0(new_n159));
  nor042aa1n04x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanp02aa1n04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  aoi112aa1n02x5               g067(.a(new_n156), .b(new_n162), .c(new_n155), .d(new_n153), .o1(new_n163));
  aoi022aa1n02x5               g068(.a(new_n159), .b(new_n162), .c(new_n152), .d(new_n163), .o1(\s[13] ));
  nor042aa1n03x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand42aa1n04x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n160), .c(new_n159), .d(new_n161), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n160), .b(new_n167), .c(new_n159), .d(new_n162), .o1(new_n169));
  norb02aa1n03x4               g074(.a(new_n168), .b(new_n169), .out0(\s[14] ));
  inv030aa1n03x5               g075(.a(new_n102), .o1(new_n171));
  nona22aa1n09x5               g076(.a(new_n171), .b(new_n107), .c(new_n103), .out0(new_n172));
  nanb03aa1n12x5               g077(.a(new_n115), .b(new_n119), .c(new_n116), .out0(new_n173));
  inv000aa1n02x5               g078(.a(new_n125), .o1(new_n174));
  aoi013aa1n03x5               g079(.a(new_n174), .b(new_n121), .c(new_n116), .d(new_n122), .o1(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n173), .c(new_n172), .d(new_n109), .o1(new_n176));
  nano23aa1n09x5               g081(.a(new_n160), .b(new_n165), .c(new_n166), .d(new_n161), .out0(new_n177));
  aoai13aa1n03x5               g082(.a(new_n177), .b(new_n158), .c(new_n176), .d(new_n151), .o1(new_n178));
  oai012aa1n18x5               g083(.a(new_n166), .b(new_n165), .c(new_n160), .o1(new_n179));
  xorc02aa1n12x5               g084(.a(\a[15] ), .b(\b[14] ), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n178), .c(new_n179), .out0(\s[15] ));
  inv000aa1d42x5               g086(.a(new_n179), .o1(new_n182));
  aoai13aa1n02x7               g087(.a(new_n180), .b(new_n182), .c(new_n159), .d(new_n177), .o1(new_n183));
  orn002aa1n02x5               g088(.a(\a[15] ), .b(\b[14] ), .o(new_n184));
  inv000aa1d42x5               g089(.a(new_n180), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n178), .d(new_n179), .o1(new_n186));
  tech160nm_fixorc02aa1n04x5   g091(.a(\a[16] ), .b(\b[15] ), .out0(new_n187));
  norb02aa1n02x5               g092(.a(new_n184), .b(new_n187), .out0(new_n188));
  aoi022aa1n02x5               g093(.a(new_n186), .b(new_n187), .c(new_n183), .d(new_n188), .o1(\s[16] ));
  xorc02aa1n02x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  nano32aa1d12x5               g095(.a(new_n150), .b(new_n187), .c(new_n177), .d(new_n180), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n192));
  inv030aa1n02x5               g097(.a(new_n192), .o1(new_n193));
  and002aa1n06x5               g098(.a(new_n187), .b(new_n180), .o(new_n194));
  aoai13aa1n04x5               g099(.a(new_n177), .b(new_n156), .c(new_n155), .d(new_n153), .o1(new_n195));
  aob012aa1n06x5               g100(.a(new_n194), .b(new_n195), .c(new_n179), .out0(new_n196));
  oao003aa1n02x5               g101(.a(\a[16] ), .b(\b[15] ), .c(new_n184), .carry(new_n197));
  nanp03aa1n09x5               g102(.a(new_n192), .b(new_n196), .c(new_n197), .o1(new_n198));
  nanp03aa1n02x5               g103(.a(new_n196), .b(new_n190), .c(new_n197), .o1(new_n199));
  obai22aa1n02x7               g104(.a(new_n198), .b(new_n190), .c(new_n199), .d(new_n193), .out0(\s[17] ));
  oai112aa1n02x5               g105(.a(new_n196), .b(new_n197), .c(\b[16] ), .d(\a[17] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n201), .b(new_n192), .out0(new_n202));
  aoi022aa1n02x5               g107(.a(\b[17] ), .b(\a[18] ), .c(\a[17] ), .d(\b[16] ), .o1(new_n203));
  oa0012aa1n02x5               g108(.a(new_n203), .b(\b[17] ), .c(\a[18] ), .o(new_n204));
  xnrc02aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .out0(new_n205));
  inv000aa1d42x5               g110(.a(\a[17] ), .o1(new_n206));
  obai22aa1n03x5               g111(.a(\b[16] ), .b(new_n206), .c(new_n201), .d(new_n193), .out0(new_n207));
  aoi022aa1n03x5               g112(.a(new_n207), .b(new_n205), .c(new_n202), .d(new_n204), .o1(\s[18] ));
  inv020aa1n04x5               g113(.a(new_n194), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n197), .b(new_n209), .c(new_n195), .d(new_n179), .o1(new_n210));
  inv000aa1n18x5               g115(.a(\a[18] ), .o1(new_n211));
  xroi22aa1d04x5               g116(.a(new_n206), .b(\b[16] ), .c(new_n211), .d(\b[17] ), .out0(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n210), .c(new_n176), .d(new_n191), .o1(new_n213));
  inv040aa1d32x5               g118(.a(\b[17] ), .o1(new_n214));
  nor042aa1n06x5               g119(.a(\b[16] ), .b(\a[17] ), .o1(new_n215));
  oao003aa1n12x5               g120(.a(new_n211), .b(new_n214), .c(new_n215), .carry(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[19] ), .b(\b[18] ), .out0(new_n218));
  xnbna2aa1n03x5               g123(.a(new_n218), .b(new_n213), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g125(.a(new_n218), .b(new_n216), .c(new_n198), .d(new_n212), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(new_n223), .b(new_n222), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n218), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(new_n213), .d(new_n217), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[20] ), .b(\b[19] ), .out0(new_n227));
  norb02aa1n02x5               g132(.a(new_n224), .b(new_n227), .out0(new_n228));
  aoi022aa1n03x5               g133(.a(new_n226), .b(new_n227), .c(new_n221), .d(new_n228), .o1(\s[20] ));
  nano23aa1n06x5               g134(.a(new_n225), .b(new_n205), .c(new_n227), .d(new_n190), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n210), .c(new_n176), .d(new_n191), .o1(new_n231));
  oaib12aa1n18x5               g136(.a(new_n216), .b(new_n223), .c(\a[19] ), .out0(new_n232));
  oa0022aa1n09x5               g137(.a(\a[20] ), .b(\b[19] ), .c(\a[19] ), .d(\b[18] ), .o(new_n233));
  aoi022aa1d18x5               g138(.a(new_n232), .b(new_n233), .c(\b[19] ), .d(\a[20] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xorc02aa1n12x5               g140(.a(\a[21] ), .b(\b[20] ), .out0(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n235), .out0(\s[21] ));
  aoai13aa1n03x5               g142(.a(new_n236), .b(new_n234), .c(new_n198), .d(new_n230), .o1(new_n238));
  nor042aa1n06x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n236), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n241), .c(new_n231), .d(new_n235), .o1(new_n242));
  nor022aa1n16x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nand02aa1n03x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoib12aa1n02x5               g150(.a(new_n239), .b(new_n244), .c(new_n243), .out0(new_n246));
  aoi022aa1n03x5               g151(.a(new_n242), .b(new_n245), .c(new_n238), .d(new_n246), .o1(\s[22] ));
  and002aa1n02x5               g152(.a(new_n236), .b(new_n245), .o(new_n248));
  and002aa1n02x5               g153(.a(new_n230), .b(new_n248), .o(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n210), .c(new_n176), .d(new_n191), .o1(new_n250));
  nand22aa1n12x5               g155(.a(new_n232), .b(new_n233), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n239), .b(\a[20] ), .c(\b[19] ), .o1(new_n252));
  aoi012aa1n06x5               g157(.a(new_n243), .b(\a[21] ), .c(\b[20] ), .o1(new_n253));
  and003aa1n02x5               g158(.a(new_n252), .b(new_n253), .c(new_n244), .o(new_n254));
  aoi112aa1n09x5               g159(.a(\b[20] ), .b(\a[21] ), .c(\a[22] ), .d(\b[21] ), .o1(new_n255));
  and002aa1n03x5               g160(.a(\b[22] ), .b(\a[23] ), .o(new_n256));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n257), .b(new_n243), .o1(new_n258));
  nona22aa1n02x4               g163(.a(new_n258), .b(new_n255), .c(new_n256), .out0(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n251), .c(new_n254), .o1(new_n260));
  nand42aa1n04x5               g165(.a(new_n250), .b(new_n260), .o1(new_n261));
  nor002aa1n02x5               g166(.a(new_n256), .b(new_n257), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n243), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n261), .b(new_n262), .c(new_n263), .d(new_n250), .o1(\s[23] ));
  nand42aa1n16x5               g169(.a(\b[23] ), .b(\a[24] ), .o1(new_n265));
  norp02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  norb02aa1n03x4               g171(.a(new_n265), .b(new_n266), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n256), .c(new_n250), .d(new_n260), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n261), .b(new_n267), .c(new_n256), .out0(new_n269));
  nanp02aa1n02x5               g174(.a(new_n269), .b(new_n268), .o1(\s[24] ));
  inv000aa1n02x5               g175(.a(new_n230), .o1(new_n271));
  and002aa1n02x5               g176(.a(new_n262), .b(new_n267), .o(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n248), .c(new_n272), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n210), .c(new_n176), .d(new_n191), .o1(new_n274));
  aoi022aa1n02x5               g179(.a(\b[23] ), .b(\a[24] ), .c(\a[23] ), .d(\b[22] ), .o1(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(\b[23] ), .c(\a[24] ), .o1(new_n276));
  oai112aa1n03x5               g181(.a(new_n253), .b(new_n244), .c(\b[22] ), .d(\a[23] ), .o1(new_n277));
  norb03aa1n12x5               g182(.a(new_n252), .b(new_n277), .c(new_n276), .out0(new_n278));
  nand02aa1d08x5               g183(.a(new_n251), .b(new_n278), .o1(new_n279));
  oai012aa1n02x5               g184(.a(new_n265), .b(new_n266), .c(new_n257), .o1(new_n280));
  oai112aa1n03x5               g185(.a(new_n262), .b(new_n267), .c(new_n255), .d(new_n243), .o1(new_n281));
  and002aa1n06x5               g186(.a(new_n281), .b(new_n280), .o(new_n282));
  nand22aa1n12x5               g187(.a(new_n279), .b(new_n282), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n274), .b(new_n284), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[25] ), .b(\b[24] ), .out0(new_n286));
  nano32aa1n02x4               g191(.a(new_n286), .b(new_n279), .c(new_n280), .d(new_n281), .out0(new_n287));
  aoi022aa1n02x5               g192(.a(new_n285), .b(new_n286), .c(new_n274), .d(new_n287), .o1(\s[25] ));
  aoai13aa1n03x5               g193(.a(new_n286), .b(new_n283), .c(new_n198), .d(new_n273), .o1(new_n289));
  orn002aa1n24x5               g194(.a(\a[25] ), .b(\b[24] ), .o(new_n290));
  inv000aa1d42x5               g195(.a(new_n286), .o1(new_n291));
  aoai13aa1n04x5               g196(.a(new_n290), .b(new_n291), .c(new_n274), .d(new_n284), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .out0(new_n293));
  norb02aa1n02x5               g198(.a(new_n290), .b(new_n293), .out0(new_n294));
  aoi022aa1n02x7               g199(.a(new_n292), .b(new_n293), .c(new_n289), .d(new_n294), .o1(\s[26] ));
  nor042aa1d18x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  norb02aa1n06x4               g202(.a(new_n297), .b(new_n296), .out0(new_n298));
  and002aa1n06x5               g203(.a(new_n293), .b(new_n286), .o(new_n299));
  nano32aa1n03x7               g204(.a(new_n271), .b(new_n299), .c(new_n248), .d(new_n272), .out0(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n210), .c(new_n176), .d(new_n191), .o1(new_n301));
  oaoi03aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n290), .o1(new_n302));
  aoi012aa1n06x5               g207(.a(new_n302), .b(new_n283), .c(new_n299), .o1(new_n303));
  inv000aa1n06x5               g208(.a(new_n282), .o1(new_n304));
  aoai13aa1n04x5               g209(.a(new_n299), .b(new_n304), .c(new_n251), .d(new_n278), .o1(new_n305));
  norb02aa1n02x5               g210(.a(new_n298), .b(new_n302), .out0(new_n306));
  nanp03aa1n02x5               g211(.a(new_n301), .b(new_n305), .c(new_n306), .o1(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n298), .c(new_n303), .d(new_n301), .o1(\s[27] ));
  nona23aa1n03x5               g213(.a(new_n301), .b(new_n305), .c(new_n302), .d(new_n296), .out0(new_n309));
  inv000aa1d42x5               g214(.a(\a[28] ), .o1(new_n310));
  inv000aa1d42x5               g215(.a(\b[27] ), .o1(new_n311));
  aob012aa1n02x5               g216(.a(new_n297), .b(\b[27] ), .c(\a[28] ), .out0(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[27] ), .b(\a[28] ), .out0(new_n314));
  nona22aa1n02x4               g219(.a(new_n305), .b(new_n302), .c(new_n296), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n297), .b(new_n315), .c(new_n198), .d(new_n300), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n316), .b(new_n314), .c(new_n309), .d(new_n313), .o1(\s[28] ));
  inv000aa1n02x5               g222(.a(new_n302), .o1(new_n318));
  nand22aa1n03x5               g223(.a(new_n305), .b(new_n318), .o1(new_n319));
  norb02aa1n06x5               g224(.a(new_n298), .b(new_n314), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n319), .c(new_n198), .d(new_n300), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  oaoi03aa1n09x5               g227(.a(new_n310), .b(new_n311), .c(new_n296), .o1(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n322), .c(new_n301), .d(new_n303), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g233(.a(new_n314), .b(new_n325), .c(new_n298), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n319), .c(new_n198), .d(new_n300), .o1(new_n330));
  inv000aa1n02x5               g235(.a(new_n329), .o1(new_n331));
  tech160nm_fioaoi03aa1n02p5x5 g236(.a(\a[29] ), .b(\b[28] ), .c(new_n323), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n332), .o1(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n331), .c(new_n301), .d(new_n303), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .out0(new_n335));
  aoi012aa1n02x5               g240(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .o1(new_n336));
  oabi12aa1n02x5               g241(.a(new_n335), .b(\a[29] ), .c(\b[28] ), .out0(new_n337));
  norp02aa1n02x5               g242(.a(new_n337), .b(new_n336), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n334), .b(new_n335), .c(new_n330), .d(new_n338), .o1(\s[30] ));
  nano22aa1n06x5               g244(.a(new_n322), .b(new_n325), .c(new_n335), .out0(new_n340));
  aoai13aa1n03x5               g245(.a(new_n340), .b(new_n319), .c(new_n198), .d(new_n300), .o1(new_n341));
  inv000aa1d42x5               g246(.a(new_n340), .o1(new_n342));
  inv000aa1d42x5               g247(.a(\a[30] ), .o1(new_n343));
  inv000aa1d42x5               g248(.a(\b[29] ), .o1(new_n344));
  oaoi03aa1n03x5               g249(.a(new_n343), .b(new_n344), .c(new_n332), .o1(new_n345));
  aoai13aa1n03x5               g250(.a(new_n345), .b(new_n342), .c(new_n301), .d(new_n303), .o1(new_n346));
  xorc02aa1n02x5               g251(.a(\a[31] ), .b(\b[30] ), .out0(new_n347));
  oabi12aa1n02x5               g252(.a(new_n347), .b(\a[30] ), .c(\b[29] ), .out0(new_n348));
  oaoi13aa1n04x5               g253(.a(new_n348), .b(new_n332), .c(new_n343), .d(new_n344), .o1(new_n349));
  aoi022aa1n03x5               g254(.a(new_n346), .b(new_n347), .c(new_n341), .d(new_n349), .o1(\s[31] ));
  norb02aa1n02x5               g255(.a(new_n105), .b(new_n108), .out0(new_n351));
  xobna2aa1n03x5               g256(.a(new_n351), .b(new_n171), .c(new_n100), .out0(\s[3] ));
  aoai13aa1n02x5               g257(.a(new_n351), .b(new_n102), .c(\a[2] ), .d(\b[1] ), .o1(new_n353));
  nanb02aa1n02x5               g258(.a(new_n106), .b(new_n104), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n105), .out0(\s[4] ));
  norb02aa1n02x5               g260(.a(new_n112), .b(new_n111), .out0(new_n356));
  inv000aa1d42x5               g261(.a(new_n112), .o1(new_n357));
  oai022aa1n02x5               g262(.a(\a[4] ), .b(\b[3] ), .c(\b[4] ), .d(\a[5] ), .o1(new_n358));
  aoi112aa1n02x5               g263(.a(new_n358), .b(new_n357), .c(new_n108), .d(new_n104), .o1(new_n359));
  oai013aa1n03x5               g264(.a(new_n359), .b(new_n102), .c(new_n107), .d(new_n103), .o1(new_n360));
  aoai13aa1n02x5               g265(.a(new_n360), .b(new_n356), .c(new_n172), .d(new_n109), .o1(\s[5] ));
  inv000aa1d42x5               g266(.a(new_n114), .o1(new_n362));
  aoi022aa1n02x5               g267(.a(new_n360), .b(new_n112), .c(new_n113), .d(new_n362), .o1(new_n363));
  nona23aa1n02x4               g268(.a(new_n360), .b(new_n113), .c(new_n357), .d(new_n114), .out0(new_n364));
  norb02aa1n02x5               g269(.a(new_n364), .b(new_n363), .out0(\s[6] ));
  xnbna2aa1n03x5               g270(.a(new_n116), .b(new_n364), .c(new_n362), .out0(\s[7] ));
  orn002aa1n02x5               g271(.a(\a[7] ), .b(\b[6] ), .o(new_n367));
  aob012aa1n02x5               g272(.a(new_n116), .b(new_n364), .c(new_n362), .out0(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n119), .b(new_n368), .c(new_n367), .out0(\s[8] ));
  nanp02aa1n02x5               g274(.a(new_n110), .b(new_n120), .o1(new_n370));
  aoi113aa1n02x5               g275(.a(new_n127), .b(new_n174), .c(new_n121), .d(new_n116), .e(new_n122), .o1(new_n371));
  aoi022aa1n02x5               g276(.a(new_n176), .b(new_n127), .c(new_n370), .d(new_n371), .o1(\s[9] ));
endmodule


