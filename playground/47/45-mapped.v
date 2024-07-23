// Benchmark "adder" written by ABC on Thu Jul 18 12:30:34 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n228, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n264, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n356, new_n357, new_n358,
    new_n360, new_n363, new_n364, new_n365, new_n367, new_n368, new_n369,
    new_n371, new_n373, new_n374;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[1] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[0] ), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oaoi13aa1n06x5               g005(.a(new_n99), .b(new_n100), .c(new_n97), .d(new_n98), .o1(new_n101));
  nor022aa1n16x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1d06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1d18x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  oaih12aa1n12x5               g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  oai012aa1n06x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  norp02aa1n12x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand42aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nona23aa1n03x5               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  nor043aa1n03x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\a[8] ), .o1(new_n117));
  oai022aa1n06x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aoi022aa1d24x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n119));
  aoai13aa1n02x7               g024(.a(new_n111), .b(new_n109), .c(new_n118), .d(new_n119), .o1(new_n120));
  oaib12aa1n03x5               g025(.a(new_n120), .b(\b[7] ), .c(new_n117), .out0(new_n121));
  nor042aa1n06x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  nand42aa1n04x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n122), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor022aa1n08x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand02aa1n06x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  norp02aa1n09x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nor042aa1n02x5               g037(.a(new_n122), .b(new_n132), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n125), .b(new_n133), .o1(new_n134));
  xobna2aa1n03x5               g039(.a(new_n131), .b(new_n134), .c(new_n128), .out0(\s[11] ));
  nor022aa1n08x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1d28x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  nano22aa1n02x4               g043(.a(new_n129), .b(new_n128), .c(new_n130), .out0(new_n139));
  aoi012aa1n02x5               g044(.a(new_n129), .b(new_n134), .c(new_n139), .o1(new_n140));
  norb03aa1n03x5               g045(.a(new_n137), .b(new_n129), .c(new_n136), .out0(new_n141));
  aob012aa1n02x5               g046(.a(new_n141), .b(new_n134), .c(new_n139), .out0(new_n142));
  oai012aa1n02x5               g047(.a(new_n142), .b(new_n140), .c(new_n138), .o1(\s[12] ));
  nano23aa1n03x5               g048(.a(new_n132), .b(new_n136), .c(new_n137), .d(new_n128), .out0(new_n144));
  nano23aa1n03x5               g049(.a(new_n122), .b(new_n129), .c(new_n130), .d(new_n123), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  inv000aa1n02x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n130), .b(new_n128), .o1(new_n149));
  tech160nm_fioai012aa1n04x5   g054(.a(new_n141), .b(new_n133), .c(new_n149), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n137), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1n10x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  inv030aa1n02x5               g060(.a(new_n152), .o1(new_n156));
  nand22aa1n03x5               g061(.a(new_n148), .b(new_n151), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(new_n157), .b(new_n154), .o1(new_n158));
  norp02aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand02aa1d16x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  oai022aa1d18x5               g066(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n162));
  nanb03aa1n03x5               g067(.a(new_n162), .b(new_n158), .c(new_n160), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n161), .c(new_n156), .d(new_n158), .o1(\s[14] ));
  nano23aa1n06x5               g069(.a(new_n152), .b(new_n159), .c(new_n160), .d(new_n153), .out0(new_n165));
  oaoi03aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .o1(new_n166));
  tech160nm_fixorc02aa1n03p5x5 g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n166), .c(new_n157), .d(new_n165), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n167), .b(new_n166), .c(new_n157), .d(new_n165), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(\s[15] ));
  inv000aa1d42x5               g075(.a(\a[15] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[14] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n171), .o1(new_n173));
  nor022aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  and002aa1n03x5               g079(.a(\b[15] ), .b(\a[16] ), .o(new_n175));
  norp02aa1n02x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  oai022aa1n02x5               g081(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n177));
  nona22aa1n02x5               g082(.a(new_n168), .b(new_n175), .c(new_n177), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n173), .d(new_n168), .o1(\s[16] ));
  nano32aa1n03x7               g084(.a(new_n146), .b(new_n176), .c(new_n165), .d(new_n167), .out0(new_n180));
  aoai13aa1n06x5               g085(.a(new_n180), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n181));
  aoi112aa1n03x5               g086(.a(new_n175), .b(new_n174), .c(\a[15] ), .d(\b[14] ), .o1(new_n182));
  oai112aa1n02x7               g087(.a(new_n153), .b(new_n160), .c(\b[14] ), .d(\a[15] ), .o1(new_n183));
  nano23aa1n06x5               g088(.a(new_n183), .b(new_n159), .c(new_n156), .d(new_n137), .out0(new_n184));
  oai112aa1n03x5               g089(.a(new_n162), .b(new_n160), .c(new_n172), .d(new_n171), .o1(new_n185));
  aoi022aa1n02x5               g090(.a(new_n185), .b(new_n173), .c(\a[16] ), .d(\b[15] ), .o1(new_n186));
  aoi113aa1n06x5               g091(.a(new_n186), .b(new_n174), .c(new_n184), .d(new_n150), .e(new_n182), .o1(new_n187));
  nanp02aa1n09x5               g092(.a(new_n181), .b(new_n187), .o1(new_n188));
  nor042aa1n06x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nand42aa1n08x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  obai22aa1n02x7               g096(.a(new_n190), .b(new_n189), .c(\a[16] ), .d(\b[15] ), .out0(new_n192));
  aoi113aa1n02x5               g097(.a(new_n192), .b(new_n186), .c(new_n184), .d(new_n150), .e(new_n182), .o1(new_n193));
  aoi022aa1n02x5               g098(.a(new_n188), .b(new_n191), .c(new_n181), .d(new_n193), .o1(\s[17] ));
  inv000aa1n06x5               g099(.a(new_n189), .o1(new_n195));
  and002aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .o(new_n196));
  oaoi03aa1n02x5               g101(.a(\a[2] ), .b(\b[1] ), .c(new_n196), .o1(new_n197));
  norb02aa1n03x5               g102(.a(new_n103), .b(new_n102), .out0(new_n198));
  norb02aa1n12x5               g103(.a(new_n105), .b(new_n104), .out0(new_n199));
  nand23aa1n03x5               g104(.a(new_n197), .b(new_n198), .c(new_n199), .o1(new_n200));
  nano23aa1n02x4               g105(.a(new_n112), .b(new_n109), .c(new_n110), .d(new_n111), .out0(new_n201));
  xorc02aa1n02x5               g106(.a(\a[5] ), .b(\b[4] ), .out0(new_n202));
  xorc02aa1n02x5               g107(.a(\a[6] ), .b(\b[5] ), .out0(new_n203));
  nanp03aa1n02x5               g108(.a(new_n201), .b(new_n202), .c(new_n203), .o1(new_n204));
  norp02aa1n02x5               g109(.a(\b[4] ), .b(\a[5] ), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[5] ), .b(\a[6] ), .o1(new_n206));
  oai012aa1n02x5               g111(.a(new_n119), .b(new_n206), .c(new_n205), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(\b[6] ), .c(\a[7] ), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n112), .b(new_n208), .c(new_n111), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n204), .c(new_n200), .d(new_n107), .o1(new_n210));
  nanp03aa1n02x5               g115(.a(new_n184), .b(new_n150), .c(new_n182), .o1(new_n211));
  nona22aa1n02x4               g116(.a(new_n211), .b(new_n186), .c(new_n174), .out0(new_n212));
  aoai13aa1n02x5               g117(.a(new_n191), .b(new_n212), .c(new_n210), .d(new_n180), .o1(new_n213));
  nor042aa1n06x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand42aa1n06x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  oai022aa1n02x5               g121(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n217));
  nanb03aa1n02x5               g122(.a(new_n217), .b(new_n213), .c(new_n215), .out0(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n216), .c(new_n195), .d(new_n213), .o1(\s[18] ));
  nano23aa1d15x5               g124(.a(new_n189), .b(new_n214), .c(new_n215), .d(new_n190), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  oaoi03aa1n02x5               g126(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n222));
  inv000aa1n02x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n221), .c(new_n181), .d(new_n187), .o1(new_n224));
  norp02aa1n02x5               g129(.a(\b[18] ), .b(\a[19] ), .o1(new_n225));
  nand42aa1n03x5               g130(.a(\b[18] ), .b(\a[19] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  aoi112aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n188), .d(new_n220), .o1(new_n228));
  tech160nm_fiaoi012aa1n02p5x5 g133(.a(new_n228), .b(new_n224), .c(new_n227), .o1(\s[19] ));
  xnrc02aa1n02x5               g134(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand02aa1n06x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  inv000aa1d42x5               g138(.a(\a[19] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\b[18] ), .o1(new_n235));
  oaoi03aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n224), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n224), .b(new_n227), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n231), .b(new_n225), .o1(new_n238));
  nanp03aa1n02x5               g143(.a(new_n237), .b(new_n232), .c(new_n238), .o1(new_n239));
  oaih12aa1n02x5               g144(.a(new_n239), .b(new_n236), .c(new_n233), .o1(\s[20] ));
  nano23aa1n03x7               g145(.a(new_n225), .b(new_n231), .c(new_n232), .d(new_n226), .out0(new_n241));
  nand22aa1n06x5               g146(.a(new_n241), .b(new_n220), .o1(new_n242));
  inv040aa1n06x5               g147(.a(new_n242), .o1(new_n243));
  nanb03aa1n02x5               g148(.a(new_n231), .b(new_n232), .c(new_n226), .out0(new_n244));
  oai112aa1n03x5               g149(.a(new_n217), .b(new_n215), .c(\b[18] ), .d(\a[19] ), .o1(new_n245));
  aoai13aa1n12x5               g150(.a(new_n232), .b(new_n231), .c(new_n234), .d(new_n235), .o1(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n246), .b(new_n245), .c(new_n244), .o1(new_n247));
  nor002aa1d32x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  nand42aa1n02x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n247), .c(new_n188), .d(new_n243), .o1(new_n251));
  nano22aa1n06x5               g156(.a(new_n231), .b(new_n226), .c(new_n232), .out0(new_n252));
  oai012aa1n02x5               g157(.a(new_n215), .b(\b[18] ), .c(\a[19] ), .o1(new_n253));
  oab012aa1n03x5               g158(.a(new_n253), .b(new_n189), .c(new_n214), .out0(new_n254));
  inv040aa1d28x5               g159(.a(new_n246), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n255), .b(new_n250), .c(new_n254), .d(new_n252), .o1(new_n256));
  aobi12aa1n02x5               g161(.a(new_n256), .b(new_n188), .c(new_n243), .out0(new_n257));
  norb02aa1n03x4               g162(.a(new_n251), .b(new_n257), .out0(\s[21] ));
  inv000aa1d42x5               g163(.a(new_n248), .o1(new_n259));
  nor042aa1n02x5               g164(.a(\b[21] ), .b(\a[22] ), .o1(new_n260));
  nand02aa1n03x5               g165(.a(\b[21] ), .b(\a[22] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  norb03aa1n02x5               g167(.a(new_n261), .b(new_n248), .c(new_n260), .out0(new_n263));
  tech160nm_finand02aa1n05x5   g168(.a(new_n251), .b(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n259), .d(new_n251), .o1(\s[22] ));
  nano23aa1d15x5               g170(.a(new_n248), .b(new_n260), .c(new_n261), .d(new_n249), .out0(new_n266));
  inv020aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  nano22aa1n02x4               g172(.a(new_n267), .b(new_n220), .c(new_n241), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n266), .b(new_n255), .c(new_n254), .d(new_n252), .o1(new_n269));
  oaoi03aa1n09x5               g174(.a(\a[22] ), .b(\b[21] ), .c(new_n259), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n269), .b(new_n271), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[23] ), .b(\b[22] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n188), .d(new_n268), .o1(new_n274));
  nona22aa1n02x4               g179(.a(new_n269), .b(new_n270), .c(new_n273), .out0(new_n275));
  aoi012aa1n02x5               g180(.a(new_n275), .b(new_n188), .c(new_n268), .o1(new_n276));
  norb02aa1n03x4               g181(.a(new_n274), .b(new_n276), .out0(\s[23] ));
  norp02aa1n02x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  tech160nm_fixorc02aa1n02p5x5 g184(.a(\a[24] ), .b(\b[23] ), .out0(new_n280));
  oai022aa1n02x5               g185(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n281));
  aoi012aa1n02x5               g186(.a(new_n281), .b(\a[24] ), .c(\b[23] ), .o1(new_n282));
  tech160nm_finand02aa1n05x5   g187(.a(new_n274), .b(new_n282), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n280), .c(new_n279), .d(new_n274), .o1(\s[24] ));
  nano32aa1n02x4               g189(.a(new_n242), .b(new_n280), .c(new_n266), .d(new_n273), .out0(new_n285));
  nand02aa1d04x5               g190(.a(new_n280), .b(new_n273), .o1(new_n286));
  aob012aa1n02x5               g191(.a(new_n281), .b(\b[23] ), .c(\a[24] ), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n286), .c(new_n269), .d(new_n271), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[25] ), .b(\b[24] ), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n288), .c(new_n188), .d(new_n285), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n286), .o1(new_n291));
  aoai13aa1n04x5               g196(.a(new_n291), .b(new_n270), .c(new_n247), .d(new_n266), .o1(new_n292));
  nanb03aa1n02x5               g197(.a(new_n289), .b(new_n292), .c(new_n287), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n188), .c(new_n285), .o1(new_n294));
  norb02aa1n03x4               g199(.a(new_n290), .b(new_n294), .out0(\s[25] ));
  nor042aa1n03x5               g200(.a(\b[24] ), .b(\a[25] ), .o1(new_n296));
  inv000aa1n03x5               g201(.a(new_n296), .o1(new_n297));
  tech160nm_fixorc02aa1n05x5   g202(.a(\a[26] ), .b(\b[25] ), .out0(new_n298));
  nanp02aa1n02x5               g203(.a(\b[25] ), .b(\a[26] ), .o1(new_n299));
  oai022aa1n02x5               g204(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n300));
  norb02aa1n02x5               g205(.a(new_n299), .b(new_n300), .out0(new_n301));
  tech160nm_finand02aa1n05x5   g206(.a(new_n290), .b(new_n301), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n297), .d(new_n290), .o1(\s[26] ));
  nanp02aa1n02x5               g208(.a(new_n298), .b(new_n289), .o1(new_n304));
  inv000aa1n02x5               g209(.a(new_n304), .o1(new_n305));
  nona23aa1n09x5               g210(.a(new_n243), .b(new_n305), .c(new_n286), .d(new_n267), .out0(new_n306));
  inv040aa1n03x5               g211(.a(new_n306), .o1(new_n307));
  aoai13aa1n06x5               g212(.a(new_n307), .b(new_n212), .c(new_n210), .d(new_n180), .o1(new_n308));
  aoi012aa1n06x5               g213(.a(new_n306), .b(new_n181), .c(new_n187), .o1(new_n309));
  tech160nm_fioaoi03aa1n02p5x5 g214(.a(\a[26] ), .b(\b[25] ), .c(new_n297), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n304), .c(new_n292), .d(new_n287), .o1(new_n312));
  xorc02aa1n03x5               g217(.a(\a[27] ), .b(\b[26] ), .out0(new_n313));
  oaih12aa1n02x5               g218(.a(new_n313), .b(new_n312), .c(new_n309), .o1(new_n314));
  aoi112aa1n02x5               g219(.a(new_n313), .b(new_n310), .c(new_n288), .d(new_n305), .o1(new_n315));
  aobi12aa1n02x7               g220(.a(new_n314), .b(new_n315), .c(new_n308), .out0(\s[27] ));
  norp02aa1n02x5               g221(.a(\b[26] ), .b(\a[27] ), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[28] ), .b(\b[27] ), .out0(new_n319));
  aoi022aa1n06x5               g224(.a(new_n288), .b(new_n305), .c(new_n299), .d(new_n300), .o1(new_n320));
  inv000aa1n03x5               g225(.a(new_n313), .o1(new_n321));
  oai022aa1n02x5               g226(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(\a[28] ), .c(\b[27] ), .o1(new_n323));
  aoai13aa1n04x5               g228(.a(new_n323), .b(new_n321), .c(new_n320), .d(new_n308), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n319), .c(new_n314), .d(new_n318), .o1(\s[28] ));
  and002aa1n02x5               g230(.a(new_n319), .b(new_n313), .o(new_n326));
  oaih12aa1n02x5               g231(.a(new_n326), .b(new_n312), .c(new_n309), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n326), .o1(new_n328));
  aob012aa1n02x5               g233(.a(new_n322), .b(\b[27] ), .c(\a[28] ), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n328), .c(new_n320), .d(new_n308), .o1(new_n330));
  tech160nm_fixorc02aa1n02p5x5 g235(.a(\a[29] ), .b(\b[28] ), .out0(new_n331));
  norb02aa1n02x5               g236(.a(new_n329), .b(new_n331), .out0(new_n332));
  aoi022aa1n03x5               g237(.a(new_n330), .b(new_n331), .c(new_n327), .d(new_n332), .o1(\s[29] ));
  xnrb03aa1n02x5               g238(.a(new_n196), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g239(.a(new_n321), .b(new_n319), .c(new_n331), .out0(new_n335));
  oaih12aa1n02x5               g240(.a(new_n335), .b(new_n312), .c(new_n309), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n335), .o1(new_n337));
  inv000aa1d42x5               g242(.a(\a[29] ), .o1(new_n338));
  inv000aa1d42x5               g243(.a(\b[28] ), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(\b[28] ), .b(\a[29] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n340));
  aoi022aa1n02x5               g245(.a(new_n322), .b(new_n340), .c(new_n339), .d(new_n338), .o1(new_n341));
  aoai13aa1n06x5               g246(.a(new_n341), .b(new_n337), .c(new_n320), .d(new_n308), .o1(new_n342));
  xorc02aa1n02x5               g247(.a(\a[30] ), .b(\b[29] ), .out0(new_n343));
  aoi122aa1n02x5               g248(.a(new_n343), .b(new_n322), .c(new_n340), .d(new_n338), .e(new_n339), .o1(new_n344));
  aoi022aa1n03x5               g249(.a(new_n342), .b(new_n343), .c(new_n336), .d(new_n344), .o1(\s[30] ));
  nano32aa1n02x4               g250(.a(new_n321), .b(new_n343), .c(new_n319), .d(new_n331), .out0(new_n346));
  oaih12aa1n02x5               g251(.a(new_n346), .b(new_n312), .c(new_n309), .o1(new_n347));
  inv000aa1n02x5               g252(.a(new_n346), .o1(new_n348));
  oao003aa1n02x5               g253(.a(\a[30] ), .b(\b[29] ), .c(new_n341), .carry(new_n349));
  aoai13aa1n03x5               g254(.a(new_n349), .b(new_n348), .c(new_n320), .d(new_n308), .o1(new_n350));
  xorc02aa1n02x5               g255(.a(\a[31] ), .b(\b[30] ), .out0(new_n351));
  nanp02aa1n02x5               g256(.a(\b[29] ), .b(\a[30] ), .o1(new_n352));
  oabi12aa1n02x5               g257(.a(new_n351), .b(\a[30] ), .c(\b[29] ), .out0(new_n353));
  aoib12aa1n02x5               g258(.a(new_n353), .b(new_n352), .c(new_n341), .out0(new_n354));
  aoi022aa1n03x5               g259(.a(new_n350), .b(new_n351), .c(new_n347), .d(new_n354), .o1(\s[31] ));
  norb03aa1n02x5               g260(.a(new_n100), .b(new_n196), .c(new_n99), .out0(new_n356));
  inv000aa1d42x5               g261(.a(new_n104), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n99), .b(new_n357), .c(new_n105), .o1(new_n358));
  aboi22aa1n03x5               g263(.a(new_n356), .b(new_n358), .c(new_n197), .d(new_n199), .out0(\s[3] ));
  nanp02aa1n02x5               g264(.a(new_n197), .b(new_n199), .o1(new_n360));
  xnbna2aa1n03x5               g265(.a(new_n198), .b(new_n360), .c(new_n357), .out0(\s[4] ));
  xnbna2aa1n03x5               g266(.a(new_n202), .b(new_n200), .c(new_n107), .out0(\s[5] ));
  aoai13aa1n02x5               g267(.a(new_n115), .b(new_n205), .c(new_n108), .d(new_n202), .o1(new_n363));
  aoi012aa1n02x5               g268(.a(new_n118), .b(\a[6] ), .c(\b[5] ), .o1(new_n364));
  aoai13aa1n02x5               g269(.a(new_n364), .b(new_n114), .c(new_n200), .d(new_n107), .o1(new_n365));
  nanp02aa1n02x5               g270(.a(new_n363), .b(new_n365), .o1(\s[6] ));
  nanb02aa1n02x5               g271(.a(new_n109), .b(new_n110), .out0(new_n367));
  aob012aa1n02x5               g272(.a(new_n365), .b(\b[5] ), .c(\a[6] ), .out0(new_n368));
  norb02aa1n02x5               g273(.a(new_n119), .b(new_n109), .out0(new_n369));
  aoi022aa1n02x5               g274(.a(new_n368), .b(new_n367), .c(new_n365), .d(new_n369), .o1(\s[7] ));
  aoi012aa1n02x5               g275(.a(new_n109), .b(new_n365), .c(new_n119), .o1(new_n371));
  xorb03aa1n02x5               g276(.a(new_n371), .b(\b[7] ), .c(new_n117), .out0(\s[8] ));
  nanp02aa1n02x5               g277(.a(new_n108), .b(new_n116), .o1(new_n373));
  aoi112aa1n02x5               g278(.a(new_n124), .b(new_n112), .c(new_n208), .d(new_n111), .o1(new_n374));
  aoi022aa1n02x5               g279(.a(new_n210), .b(new_n124), .c(new_n373), .d(new_n374), .o1(\s[9] ));
endmodule


