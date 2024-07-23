// Benchmark "adder" written by ABC on Thu Jul 18 06:50:50 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n352, new_n354, new_n356, new_n358, new_n359, new_n360, new_n362,
    new_n363, new_n364, new_n366, new_n368;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor042aa1n04x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  nanp02aa1n04x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nanp02aa1n12x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nand02aa1n08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor002aa1d24x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n06x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  nor002aa1n12x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n06x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1d12x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  aoi012aa1d24x5               g014(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n110));
  nand02aa1n16x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor042aa1n03x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n04x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nano23aa1n09x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  nor042aa1n04x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand22aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1n20x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nor002aa1d24x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n119), .b(new_n116), .c(new_n117), .d(new_n118), .out0(new_n120));
  nanp02aa1n03x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  norb02aa1n12x5               g026(.a(new_n117), .b(new_n116), .out0(new_n122));
  nano22aa1n12x5               g027(.a(new_n119), .b(new_n111), .c(new_n118), .out0(new_n123));
  oaih22aa1n06x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  inv040aa1n03x5               g029(.a(new_n119), .o1(new_n125));
  oaoi03aa1n12x5               g030(.a(\a[8] ), .b(\b[7] ), .c(new_n125), .o1(new_n126));
  aoi013aa1n09x5               g031(.a(new_n126), .b(new_n123), .c(new_n124), .d(new_n122), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n127), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n128));
  tech160nm_fixorc02aa1n05x5   g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  nand02aa1d06x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  xorc02aa1n12x5               g035(.a(\a[10] ), .b(\b[9] ), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n130), .c(new_n97), .out0(\s[10] ));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nor002aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  nand42aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  oaih22aa1d12x5               g041(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  nand02aa1n02x5               g043(.a(new_n130), .b(new_n138), .o1(new_n139));
  aoi022aa1n02x5               g044(.a(new_n139), .b(new_n133), .c(new_n135), .d(new_n136), .o1(new_n140));
  nanb03aa1n02x5               g045(.a(new_n134), .b(new_n136), .c(new_n133), .out0(new_n141));
  nanb02aa1n02x5               g046(.a(new_n141), .b(new_n139), .out0(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n140), .out0(\s[11] ));
  aoai13aa1n02x5               g048(.a(new_n135), .b(new_n141), .c(new_n130), .d(new_n138), .o1(new_n144));
  norp02aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n09x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  aoib12aa1n02x5               g052(.a(new_n134), .b(new_n146), .c(new_n145), .out0(new_n148));
  aoi022aa1n02x5               g053(.a(new_n142), .b(new_n148), .c(new_n144), .d(new_n147), .o1(\s[12] ));
  aoi012aa1n02x5               g054(.a(new_n98), .b(new_n100), .c(new_n101), .o1(new_n150));
  nona23aa1n03x5               g055(.a(new_n103), .b(new_n107), .c(new_n106), .d(new_n104), .out0(new_n151));
  oai012aa1n04x7               g056(.a(new_n110), .b(new_n151), .c(new_n150), .o1(new_n152));
  nanb02aa1n06x5               g057(.a(new_n121), .b(new_n152), .out0(new_n153));
  nano23aa1n06x5               g058(.a(new_n134), .b(new_n145), .c(new_n146), .d(new_n136), .out0(new_n154));
  nand23aa1n03x5               g059(.a(new_n154), .b(new_n129), .c(new_n131), .o1(new_n155));
  aoi022aa1n02x5               g060(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n156));
  norp02aa1n02x5               g061(.a(new_n145), .b(new_n134), .o1(new_n157));
  aob012aa1n03x5               g062(.a(new_n157), .b(new_n137), .c(new_n156), .out0(new_n158));
  and002aa1n06x5               g063(.a(new_n158), .b(new_n146), .o(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n155), .c(new_n153), .d(new_n127), .o1(new_n161));
  tech160nm_fixorc02aa1n02p5x5 g066(.a(\a[13] ), .b(\b[12] ), .out0(new_n162));
  inv000aa1n06x5               g067(.a(new_n155), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n162), .b(new_n159), .c(new_n128), .d(new_n163), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n162), .o1(\s[13] ));
  nor042aa1n06x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n162), .b(new_n159), .c(new_n128), .d(new_n163), .o1(new_n168));
  xorc02aa1n02x5               g073(.a(\a[14] ), .b(\b[13] ), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n168), .c(new_n167), .out0(\s[14] ));
  inv000aa1d42x5               g075(.a(\a[13] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\a[14] ), .o1(new_n172));
  xroi22aa1d04x5               g077(.a(new_n171), .b(\b[12] ), .c(new_n172), .d(\b[13] ), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n159), .c(new_n128), .d(new_n163), .o1(new_n174));
  tech160nm_fioaoi03aa1n03p5x5 g079(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  nor002aa1n20x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  and002aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o(new_n178));
  nor042aa1n02x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n174), .c(new_n176), .out0(\s[15] ));
  aoai13aa1n02x5               g085(.a(new_n179), .b(new_n175), .c(new_n161), .d(new_n173), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n177), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n178), .c(new_n174), .d(new_n176), .o1(new_n183));
  orn002aa1n24x5               g088(.a(\a[16] ), .b(\b[15] ), .o(new_n184));
  tech160nm_finand02aa1n05x5   g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nand22aa1n09x5               g090(.a(new_n184), .b(new_n185), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[15] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[14] ), .o1(new_n189));
  aoi022aa1n02x5               g094(.a(new_n184), .b(new_n185), .c(new_n189), .d(new_n188), .o1(new_n190));
  aoi022aa1n02x7               g095(.a(new_n183), .b(new_n187), .c(new_n181), .d(new_n190), .o1(\s[16] ));
  nor043aa1n02x5               g096(.a(new_n186), .b(new_n178), .c(new_n177), .o1(new_n192));
  nano22aa1n06x5               g097(.a(new_n155), .b(new_n173), .c(new_n192), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n128), .b(new_n193), .o1(new_n194));
  nano32aa1n02x4               g099(.a(new_n186), .b(new_n169), .c(new_n162), .d(new_n179), .out0(new_n195));
  nand02aa1n02x5               g100(.a(new_n163), .b(new_n195), .o1(new_n196));
  nano22aa1n02x4               g101(.a(new_n178), .b(new_n184), .c(new_n185), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(\b[13] ), .b(\a[14] ), .o1(new_n198));
  oai012aa1n02x5               g103(.a(new_n146), .b(\b[12] ), .c(\a[13] ), .o1(new_n199));
  obai22aa1n02x7               g104(.a(\b[12] ), .b(new_n171), .c(\b[13] ), .d(\a[14] ), .out0(new_n200));
  nano23aa1n03x7               g105(.a(new_n200), .b(new_n199), .c(new_n182), .d(new_n198), .out0(new_n201));
  oai022aa1n02x5               g106(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n202));
  oai112aa1n02x5               g107(.a(new_n202), .b(new_n198), .c(new_n189), .d(new_n188), .o1(new_n203));
  oab012aa1n02x4               g108(.a(new_n177), .b(\a[16] ), .c(\b[15] ), .out0(new_n204));
  aoi022aa1n02x5               g109(.a(new_n203), .b(new_n204), .c(\b[15] ), .d(\a[16] ), .o1(new_n205));
  aoi013aa1n02x4               g110(.a(new_n205), .b(new_n201), .c(new_n158), .d(new_n197), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n196), .c(new_n153), .d(new_n127), .o1(new_n207));
  xorc02aa1n12x5               g112(.a(\a[17] ), .b(\b[16] ), .out0(new_n208));
  aoi113aa1n02x5               g113(.a(new_n208), .b(new_n205), .c(new_n201), .d(new_n158), .e(new_n197), .o1(new_n209));
  aoi022aa1n02x5               g114(.a(new_n207), .b(new_n208), .c(new_n194), .d(new_n209), .o1(\s[17] ));
  nor042aa1n03x5               g115(.a(\b[16] ), .b(\a[17] ), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nanp03aa1n02x5               g117(.a(new_n201), .b(new_n158), .c(new_n197), .o1(new_n213));
  nanb02aa1n06x5               g118(.a(new_n205), .b(new_n213), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n208), .b(new_n214), .c(new_n128), .d(new_n193), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n215), .c(new_n212), .out0(\s[18] ));
  xnrc02aa1n02x5               g122(.a(\b[17] ), .b(\a[18] ), .out0(new_n218));
  norb02aa1n06x5               g123(.a(new_n208), .b(new_n218), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n214), .c(new_n128), .d(new_n193), .o1(new_n220));
  oaoi03aa1n12x5               g125(.a(\a[18] ), .b(\b[17] ), .c(new_n212), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xorc02aa1n12x5               g127(.a(\a[19] ), .b(\b[18] ), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n220), .c(new_n222), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g130(.a(new_n223), .b(new_n221), .c(new_n207), .d(new_n219), .o1(new_n226));
  norp02aa1n02x5               g131(.a(\b[18] ), .b(\a[19] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n223), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n229), .c(new_n220), .d(new_n222), .o1(new_n230));
  norp02aa1n02x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand42aa1n02x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  norb02aa1n06x4               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  aoib12aa1n02x5               g138(.a(new_n227), .b(new_n232), .c(new_n231), .out0(new_n234));
  aoi022aa1n03x5               g139(.a(new_n230), .b(new_n233), .c(new_n226), .d(new_n234), .o1(\s[20] ));
  nand23aa1d12x5               g140(.a(new_n219), .b(new_n223), .c(new_n233), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n214), .c(new_n128), .d(new_n193), .o1(new_n238));
  oaih22aa1d12x5               g143(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n239));
  aoi022aa1d24x5               g144(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n240));
  oai022aa1d18x5               g145(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n241));
  aoi012aa1d18x5               g146(.a(new_n241), .b(new_n239), .c(new_n240), .o1(new_n242));
  norb02aa1n03x5               g147(.a(new_n232), .b(new_n242), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[21] ), .b(\b[20] ), .out0(new_n245));
  xnbna2aa1n03x5               g150(.a(new_n245), .b(new_n238), .c(new_n244), .out0(\s[21] ));
  aoai13aa1n03x5               g151(.a(new_n245), .b(new_n243), .c(new_n207), .d(new_n237), .o1(new_n247));
  nor002aa1d32x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n245), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n249), .b(new_n250), .c(new_n238), .d(new_n244), .o1(new_n251));
  nor002aa1d32x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  nand22aa1n06x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  norb02aa1n06x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  aoib12aa1n02x5               g159(.a(new_n248), .b(new_n253), .c(new_n252), .out0(new_n255));
  aoi022aa1n02x7               g160(.a(new_n251), .b(new_n254), .c(new_n247), .d(new_n255), .o1(\s[22] ));
  and002aa1n06x5               g161(.a(new_n245), .b(new_n254), .o(new_n257));
  norb02aa1n02x7               g162(.a(new_n257), .b(new_n236), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n214), .c(new_n128), .d(new_n193), .o1(new_n259));
  tech160nm_fiaoi012aa1n04x5   g164(.a(new_n248), .b(\a[20] ), .c(\b[19] ), .o1(new_n260));
  tech160nm_fiaoi012aa1n05x5   g165(.a(new_n252), .b(\a[21] ), .c(\b[20] ), .o1(new_n261));
  nand23aa1n06x5               g166(.a(new_n260), .b(new_n261), .c(new_n253), .o1(new_n262));
  aoi012aa1d24x5               g167(.a(new_n252), .b(new_n248), .c(new_n253), .o1(new_n263));
  oai012aa1d24x5               g168(.a(new_n263), .b(new_n262), .c(new_n242), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n259), .b(new_n265), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[23] ), .b(\b[22] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  oai112aa1n02x5               g173(.a(new_n263), .b(new_n268), .c(new_n262), .d(new_n242), .o1(new_n269));
  aboi22aa1n03x5               g174(.a(new_n269), .b(new_n259), .c(new_n266), .d(new_n267), .out0(\s[23] ));
  aoai13aa1n03x5               g175(.a(new_n267), .b(new_n264), .c(new_n207), .d(new_n258), .o1(new_n271));
  nor002aa1d24x5               g176(.a(\b[22] ), .b(\a[23] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n259), .d(new_n265), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[24] ), .b(\b[23] ), .out0(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n272), .o1(new_n276));
  aoi022aa1n03x5               g181(.a(new_n274), .b(new_n275), .c(new_n271), .d(new_n276), .o1(\s[24] ));
  nanp02aa1n02x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[23] ), .b(\a[24] ), .out0(new_n279));
  nano22aa1n03x7               g184(.a(new_n279), .b(new_n273), .c(new_n278), .out0(new_n280));
  nano22aa1n02x5               g185(.a(new_n236), .b(new_n257), .c(new_n280), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n214), .c(new_n128), .d(new_n193), .o1(new_n282));
  oaoi03aa1n12x5               g187(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .o1(new_n283));
  aoi012aa1d24x5               g188(.a(new_n283), .b(new_n264), .c(new_n280), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n282), .b(new_n284), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[25] ), .b(\b[24] ), .out0(new_n286));
  nanp02aa1n02x5               g191(.a(new_n275), .b(new_n267), .o1(new_n287));
  oaoi13aa1n02x7               g192(.a(new_n287), .b(new_n263), .c(new_n262), .d(new_n242), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n283), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n286), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoi022aa1n02x5               g196(.a(new_n285), .b(new_n286), .c(new_n282), .d(new_n291), .o1(\s[25] ));
  inv000aa1d42x5               g197(.a(new_n284), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n286), .b(new_n293), .c(new_n207), .d(new_n281), .o1(new_n294));
  nor042aa1n06x5               g199(.a(\b[24] ), .b(\a[25] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n290), .c(new_n282), .d(new_n284), .o1(new_n297));
  tech160nm_fixorc02aa1n02p5x5 g202(.a(\a[26] ), .b(\b[25] ), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n295), .o1(new_n299));
  aoi022aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[26] ));
  and002aa1n06x5               g205(.a(new_n298), .b(new_n286), .o(new_n301));
  nano32aa1n03x7               g206(.a(new_n236), .b(new_n301), .c(new_n257), .d(new_n280), .out0(new_n302));
  aoai13aa1n04x5               g207(.a(new_n301), .b(new_n283), .c(new_n264), .d(new_n280), .o1(new_n303));
  oaoi03aa1n09x5               g208(.a(\a[26] ), .b(\b[25] ), .c(new_n296), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n304), .o1(new_n305));
  tech160nm_finand02aa1n03p5x5 g210(.a(new_n303), .b(new_n305), .o1(new_n306));
  xorc02aa1n12x5               g211(.a(\a[27] ), .b(\b[26] ), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n207), .d(new_n302), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n302), .b(new_n214), .c(new_n128), .d(new_n193), .o1(new_n309));
  nano32aa1n03x7               g214(.a(new_n307), .b(new_n309), .c(new_n303), .d(new_n305), .out0(new_n310));
  norb02aa1n03x4               g215(.a(new_n308), .b(new_n310), .out0(\s[27] ));
  oaoi13aa1n04x5               g216(.a(new_n304), .b(new_n301), .c(new_n288), .d(new_n283), .o1(new_n312));
  norp02aa1n02x5               g217(.a(\b[26] ), .b(\a[27] ), .o1(new_n313));
  inv000aa1n03x5               g218(.a(new_n313), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n307), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n314), .b(new_n315), .c(new_n309), .d(new_n312), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .out0(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n313), .o1(new_n318));
  aoi022aa1n02x7               g223(.a(new_n316), .b(new_n317), .c(new_n308), .d(new_n318), .o1(\s[28] ));
  and002aa1n02x5               g224(.a(new_n317), .b(new_n307), .o(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n306), .c(new_n207), .d(new_n302), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[28] ), .b(\b[27] ), .c(new_n314), .carry(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n322), .c(new_n309), .d(new_n312), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n09x5               g233(.a(new_n315), .b(new_n317), .c(new_n325), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n306), .c(new_n207), .d(new_n302), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n329), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[28] ), .o1(new_n332));
  inv000aa1d42x5               g237(.a(\a[29] ), .o1(new_n333));
  oaib12aa1n02x5               g238(.a(new_n323), .b(\b[28] ), .c(new_n333), .out0(new_n334));
  oaib12aa1n02x5               g239(.a(new_n334), .b(new_n332), .c(\a[29] ), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n331), .c(new_n309), .d(new_n312), .o1(new_n336));
  xorc02aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .out0(new_n337));
  oaoi13aa1n02x5               g242(.a(new_n337), .b(new_n334), .c(new_n333), .d(new_n332), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n336), .b(new_n337), .c(new_n330), .d(new_n338), .o1(\s[30] ));
  nano32aa1n03x7               g244(.a(new_n315), .b(new_n337), .c(new_n317), .d(new_n325), .out0(new_n340));
  aoai13aa1n03x5               g245(.a(new_n340), .b(new_n306), .c(new_n207), .d(new_n302), .o1(new_n341));
  aoi022aa1n02x5               g246(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n342));
  norb02aa1n02x5               g247(.a(\b[30] ), .b(\a[31] ), .out0(new_n343));
  obai22aa1n02x7               g248(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n344));
  aoi112aa1n02x5               g249(.a(new_n344), .b(new_n343), .c(new_n334), .d(new_n342), .o1(new_n345));
  xorc02aa1n02x5               g250(.a(\a[31] ), .b(\b[30] ), .out0(new_n346));
  inv000aa1d42x5               g251(.a(new_n340), .o1(new_n347));
  norp02aa1n02x5               g252(.a(\b[29] ), .b(\a[30] ), .o1(new_n348));
  aoi012aa1n02x5               g253(.a(new_n348), .b(new_n334), .c(new_n342), .o1(new_n349));
  aoai13aa1n03x5               g254(.a(new_n349), .b(new_n347), .c(new_n309), .d(new_n312), .o1(new_n350));
  aoi022aa1n03x5               g255(.a(new_n350), .b(new_n346), .c(new_n341), .d(new_n345), .o1(\s[31] ));
  aoi112aa1n02x5               g256(.a(new_n108), .b(new_n98), .c(new_n100), .d(new_n101), .o1(new_n352));
  aoi012aa1n02x5               g257(.a(new_n352), .b(new_n102), .c(new_n108), .o1(\s[3] ));
  aoi112aa1n02x5               g258(.a(new_n106), .b(new_n105), .c(new_n102), .d(new_n107), .o1(new_n354));
  aoib12aa1n02x5               g259(.a(new_n354), .b(new_n152), .c(new_n104), .out0(\s[4] ));
  norb02aa1n02x5               g260(.a(new_n114), .b(new_n113), .out0(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n356), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  norb02aa1n02x5               g262(.a(new_n111), .b(new_n112), .out0(new_n358));
  aoai13aa1n02x5               g263(.a(new_n358), .b(new_n113), .c(new_n152), .d(new_n114), .o1(new_n359));
  aoi112aa1n02x5               g264(.a(new_n113), .b(new_n358), .c(new_n152), .d(new_n356), .o1(new_n360));
  norb02aa1n02x5               g265(.a(new_n359), .b(new_n360), .out0(\s[6] ));
  norb02aa1n02x5               g266(.a(new_n118), .b(new_n119), .out0(new_n362));
  oai012aa1n02x5               g267(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n363));
  nanp02aa1n02x5               g268(.a(new_n152), .b(new_n115), .o1(new_n364));
  xnbna2aa1n03x5               g269(.a(new_n362), .b(new_n364), .c(new_n363), .out0(\s[7] ));
  aob012aa1n02x5               g270(.a(new_n362), .b(new_n364), .c(new_n363), .out0(new_n366));
  xnbna2aa1n03x5               g271(.a(new_n122), .b(new_n366), .c(new_n125), .out0(\s[8] ));
  aoi113aa1n02x5               g272(.a(new_n129), .b(new_n126), .c(new_n123), .d(new_n124), .e(new_n122), .o1(new_n368));
  aoi022aa1n02x5               g273(.a(new_n128), .b(new_n129), .c(new_n153), .d(new_n368), .o1(\s[9] ));
endmodule


