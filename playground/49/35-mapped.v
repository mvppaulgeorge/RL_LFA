// Benchmark "adder" written by ABC on Thu Jul 18 14:52:41 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n346, new_n348, new_n349, new_n352, new_n353, new_n354,
    new_n355, new_n357, new_n359, new_n360, new_n362;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n09x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nona22aa1n09x5               g006(.a(new_n101), .b(new_n100), .c(new_n99), .out0(new_n102));
  nor042aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  tech160nm_fiaoi012aa1n04x5   g008(.a(new_n103), .b(\a[2] ), .c(\b[1] ), .o1(new_n104));
  norp02aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand02aa1d10x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1n20x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nano22aa1n03x7               g012(.a(new_n105), .b(new_n106), .c(new_n107), .out0(new_n108));
  nand23aa1n04x5               g013(.a(new_n108), .b(new_n102), .c(new_n104), .o1(new_n109));
  aoi012aa1n06x5               g014(.a(new_n105), .b(new_n103), .c(new_n106), .o1(new_n110));
  nand22aa1n03x5               g015(.a(new_n109), .b(new_n110), .o1(new_n111));
  xorc02aa1n12x5               g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  norp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  and002aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o(new_n115));
  nor042aa1n03x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nor042aa1n04x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  and002aa1n12x5               g022(.a(\b[6] ), .b(\a[7] ), .o(new_n118));
  nona22aa1n02x4               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  nano22aa1n06x5               g024(.a(new_n119), .b(new_n112), .c(new_n113), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  oai022aa1n04x7               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  aoi112aa1n09x5               g027(.a(new_n118), .b(new_n117), .c(\a[6] ), .d(\b[5] ), .o1(new_n123));
  oai022aa1n02x5               g028(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  aoai13aa1n06x5               g029(.a(new_n121), .b(new_n124), .c(new_n123), .d(new_n122), .o1(new_n125));
  inv040aa1n02x5               g030(.a(new_n125), .o1(new_n126));
  tech160nm_fixorc02aa1n05x5   g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n111), .d(new_n120), .o1(new_n128));
  nor002aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n03x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  nona22aa1n06x5               g037(.a(new_n128), .b(new_n129), .c(new_n97), .out0(new_n133));
  nand42aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor042aa1n09x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb03aa1n02x5               g040(.a(new_n135), .b(new_n130), .c(new_n134), .out0(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n134), .out0(new_n137));
  xnrc02aa1n03x5               g042(.a(\b[7] ), .b(\a[8] ), .out0(new_n138));
  tech160nm_fixnrc02aa1n02p5x5 g043(.a(\b[6] ), .b(\a[7] ), .out0(new_n139));
  nona23aa1n09x5               g044(.a(new_n116), .b(new_n112), .c(new_n139), .d(new_n138), .out0(new_n140));
  aoai13aa1n12x5               g045(.a(new_n125), .b(new_n140), .c(new_n109), .d(new_n110), .o1(new_n141));
  oai022aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n130), .b(new_n142), .c(new_n141), .d(new_n127), .o1(new_n143));
  aboi22aa1n03x5               g048(.a(new_n136), .b(new_n133), .c(new_n143), .d(new_n137), .out0(\s[11] ));
  obai22aa1n02x5               g049(.a(new_n133), .b(new_n136), .c(\a[11] ), .d(\b[10] ), .out0(new_n145));
  nor042aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand02aa1d08x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  aoi113aa1n02x5               g053(.a(new_n148), .b(new_n135), .c(new_n133), .d(new_n134), .e(new_n130), .o1(new_n149));
  tech160nm_fiaoi012aa1n02p5x5 g054(.a(new_n149), .b(new_n145), .c(new_n148), .o1(\s[12] ));
  nano32aa1n02x5               g055(.a(new_n137), .b(new_n127), .c(new_n148), .d(new_n131), .out0(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n126), .c(new_n111), .d(new_n120), .o1(new_n152));
  nano22aa1n03x7               g057(.a(new_n146), .b(new_n134), .c(new_n147), .out0(new_n153));
  oaih12aa1n02x5               g058(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .o1(new_n154));
  oab012aa1n04x5               g059(.a(new_n154), .b(new_n97), .c(new_n129), .out0(new_n155));
  tech160nm_fiaoi012aa1n04x5   g060(.a(new_n146), .b(new_n135), .c(new_n147), .o1(new_n156));
  aob012aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n153), .out0(new_n157));
  nanb02aa1n03x5               g062(.a(new_n157), .b(new_n152), .out0(new_n158));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand02aa1n06x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  inv030aa1n04x5               g066(.a(new_n156), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n155), .d(new_n153), .o1(new_n163));
  aoi022aa1n02x5               g068(.a(new_n158), .b(new_n161), .c(new_n152), .d(new_n163), .o1(\s[13] ));
  nor042aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand02aa1n08x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n159), .c(new_n158), .d(new_n160), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n159), .b(new_n167), .c(new_n158), .d(new_n161), .o1(new_n169));
  norb02aa1n03x4               g074(.a(new_n168), .b(new_n169), .out0(\s[14] ));
  nano23aa1d15x5               g075(.a(new_n159), .b(new_n165), .c(new_n166), .d(new_n160), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n157), .c(new_n141), .d(new_n151), .o1(new_n172));
  oaih12aa1n12x5               g077(.a(new_n166), .b(new_n165), .c(new_n159), .o1(new_n173));
  xorc02aa1n12x5               g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  inv000aa1d42x5               g080(.a(new_n173), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n174), .b(new_n176), .c(new_n158), .d(new_n171), .o1(new_n177));
  nor042aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n174), .o1(new_n180));
  aoai13aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n172), .d(new_n173), .o1(new_n181));
  xorc02aa1n12x5               g086(.a(\a[16] ), .b(\b[15] ), .out0(new_n182));
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n178), .o1(new_n183));
  aoi022aa1n03x5               g088(.a(new_n181), .b(new_n182), .c(new_n177), .d(new_n183), .o1(\s[16] ));
  nano23aa1n06x5               g089(.a(new_n146), .b(new_n135), .c(new_n147), .d(new_n134), .out0(new_n185));
  nand23aa1n06x5               g090(.a(new_n171), .b(new_n174), .c(new_n182), .o1(new_n186));
  nano32aa1d12x5               g091(.a(new_n186), .b(new_n185), .c(new_n131), .d(new_n127), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n126), .c(new_n111), .d(new_n120), .o1(new_n188));
  and002aa1n02x5               g093(.a(new_n182), .b(new_n174), .o(new_n189));
  aoai13aa1n12x5               g094(.a(new_n171), .b(new_n162), .c(new_n155), .d(new_n153), .o1(new_n190));
  aob012aa1n06x5               g095(.a(new_n189), .b(new_n190), .c(new_n173), .out0(new_n191));
  oao003aa1n02x5               g096(.a(\a[16] ), .b(\b[15] ), .c(new_n179), .carry(new_n192));
  nand23aa1n06x5               g097(.a(new_n188), .b(new_n191), .c(new_n192), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  nano22aa1n02x4               g099(.a(new_n194), .b(new_n191), .c(new_n192), .out0(new_n195));
  aoi022aa1n02x5               g100(.a(new_n195), .b(new_n188), .c(new_n193), .d(new_n194), .o1(\s[17] ));
  nor002aa1d32x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n182), .b(new_n174), .o1(new_n199));
  aoai13aa1n09x5               g104(.a(new_n192), .b(new_n199), .c(new_n190), .d(new_n173), .o1(new_n200));
  aoai13aa1n03x5               g105(.a(new_n194), .b(new_n200), .c(new_n141), .d(new_n187), .o1(new_n201));
  nor002aa1d32x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand42aa1d28x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb02aa1n06x4               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n201), .c(new_n198), .out0(\s[18] ));
  and002aa1n02x5               g110(.a(new_n194), .b(new_n204), .o(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n200), .c(new_n141), .d(new_n187), .o1(new_n207));
  oaoi03aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n198), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nor042aa1d18x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n20x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n12x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g119(.a(new_n212), .b(new_n208), .c(new_n193), .d(new_n206), .o1(new_n215));
  inv040aa1n02x5               g120(.a(new_n210), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n212), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n216), .b(new_n217), .c(new_n207), .d(new_n209), .o1(new_n218));
  nor002aa1n12x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand42aa1d28x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  aboi22aa1n03x5               g128(.a(new_n219), .b(new_n220), .c(new_n222), .d(new_n223), .out0(new_n224));
  aoi022aa1n03x5               g129(.a(new_n218), .b(new_n221), .c(new_n215), .d(new_n224), .o1(\s[20] ));
  nano32aa1n03x7               g130(.a(new_n217), .b(new_n194), .c(new_n221), .d(new_n204), .out0(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n200), .c(new_n141), .d(new_n187), .o1(new_n227));
  nanb03aa1n09x5               g132(.a(new_n219), .b(new_n220), .c(new_n211), .out0(new_n228));
  oai112aa1n06x5               g133(.a(new_n216), .b(new_n203), .c(new_n202), .d(new_n197), .o1(new_n229));
  aoi012aa1n12x5               g134(.a(new_n219), .b(new_n210), .c(new_n220), .o1(new_n230));
  oai012aa1d24x5               g135(.a(new_n230), .b(new_n229), .c(new_n228), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  nanp02aa1n03x5               g137(.a(new_n227), .b(new_n232), .o1(new_n233));
  nor042aa1d18x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nand42aa1n20x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n09x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  nano22aa1n02x5               g141(.a(new_n219), .b(new_n211), .c(new_n220), .out0(new_n237));
  oai012aa1n02x7               g142(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .o1(new_n238));
  oab012aa1n04x5               g143(.a(new_n238), .b(new_n197), .c(new_n202), .out0(new_n239));
  inv020aa1n03x5               g144(.a(new_n230), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n240), .b(new_n236), .c(new_n239), .d(new_n237), .o1(new_n241));
  aoi022aa1n02x5               g146(.a(new_n233), .b(new_n236), .c(new_n227), .d(new_n241), .o1(\s[21] ));
  aoai13aa1n03x5               g147(.a(new_n236), .b(new_n231), .c(new_n193), .d(new_n226), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n234), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n236), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n245), .c(new_n227), .d(new_n232), .o1(new_n246));
  nor042aa1n09x5               g151(.a(\b[21] ), .b(\a[22] ), .o1(new_n247));
  nand42aa1d28x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n247), .out0(new_n249));
  aoib12aa1n02x5               g154(.a(new_n234), .b(new_n248), .c(new_n247), .out0(new_n250));
  aoi022aa1n03x5               g155(.a(new_n246), .b(new_n249), .c(new_n243), .d(new_n250), .o1(\s[22] ));
  inv000aa1n02x5               g156(.a(new_n226), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n252), .b(new_n236), .c(new_n249), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n200), .c(new_n141), .d(new_n187), .o1(new_n254));
  nano23aa1d15x5               g159(.a(new_n234), .b(new_n247), .c(new_n248), .d(new_n235), .out0(new_n255));
  aoi012aa1d18x5               g160(.a(new_n247), .b(new_n234), .c(new_n248), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoi012aa1d24x5               g162(.a(new_n257), .b(new_n231), .c(new_n255), .o1(new_n258));
  nanp02aa1n03x5               g163(.a(new_n254), .b(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoi112aa1n02x5               g165(.a(new_n260), .b(new_n257), .c(new_n231), .d(new_n255), .o1(new_n261));
  aoi022aa1n02x5               g166(.a(new_n259), .b(new_n260), .c(new_n254), .d(new_n261), .o1(\s[23] ));
  inv000aa1d42x5               g167(.a(new_n258), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n260), .b(new_n263), .c(new_n193), .d(new_n253), .o1(new_n264));
  nor042aa1n06x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n260), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n266), .b(new_n267), .c(new_n254), .d(new_n258), .o1(new_n268));
  tech160nm_fixorc02aa1n02p5x5 g173(.a(\a[24] ), .b(\b[23] ), .out0(new_n269));
  norp02aa1n02x5               g174(.a(new_n269), .b(new_n265), .o1(new_n270));
  aoi022aa1n02x7               g175(.a(new_n268), .b(new_n269), .c(new_n264), .d(new_n270), .o1(\s[24] ));
  and002aa1n12x5               g176(.a(new_n269), .b(new_n260), .o(new_n272));
  nano22aa1n02x4               g177(.a(new_n252), .b(new_n272), .c(new_n255), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n200), .c(new_n141), .d(new_n187), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n255), .b(new_n240), .c(new_n239), .d(new_n237), .o1(new_n275));
  inv000aa1n06x5               g180(.a(new_n272), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .carry(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .d(new_n256), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  nanp02aa1n03x5               g184(.a(new_n274), .b(new_n279), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n272), .b(new_n257), .c(new_n231), .d(new_n255), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n281), .o1(new_n283));
  and003aa1n02x5               g188(.a(new_n282), .b(new_n283), .c(new_n277), .o(new_n284));
  aoi022aa1n02x5               g189(.a(new_n280), .b(new_n281), .c(new_n274), .d(new_n284), .o1(\s[25] ));
  aoai13aa1n03x5               g190(.a(new_n281), .b(new_n278), .c(new_n193), .d(new_n273), .o1(new_n286));
  nor042aa1n03x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n283), .c(new_n274), .d(new_n279), .o1(new_n289));
  tech160nm_fixorc02aa1n03p5x5 g194(.a(\a[26] ), .b(\b[25] ), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n287), .o1(new_n291));
  aoi022aa1n02x7               g196(.a(new_n289), .b(new_n290), .c(new_n286), .d(new_n291), .o1(\s[26] ));
  and002aa1n12x5               g197(.a(new_n290), .b(new_n281), .o(new_n293));
  nano32aa1n03x7               g198(.a(new_n252), .b(new_n293), .c(new_n255), .d(new_n272), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n200), .c(new_n141), .d(new_n187), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n288), .carry(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  aoi012aa1d18x5               g202(.a(new_n297), .b(new_n278), .c(new_n293), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n295), .b(new_n298), .o1(new_n299));
  xorc02aa1n12x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  aoi112aa1n02x5               g205(.a(new_n300), .b(new_n297), .c(new_n278), .d(new_n293), .o1(new_n301));
  aoi022aa1n02x5               g206(.a(new_n299), .b(new_n300), .c(new_n295), .d(new_n301), .o1(\s[27] ));
  inv000aa1d42x5               g207(.a(new_n293), .o1(new_n303));
  aoai13aa1n04x5               g208(.a(new_n296), .b(new_n303), .c(new_n282), .d(new_n277), .o1(new_n304));
  aoai13aa1n02x5               g209(.a(new_n300), .b(new_n304), .c(new_n193), .d(new_n294), .o1(new_n305));
  norp02aa1n02x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  inv000aa1n03x5               g211(.a(new_n306), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n300), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n307), .b(new_n308), .c(new_n295), .d(new_n298), .o1(new_n309));
  tech160nm_fixorc02aa1n02p5x5 g214(.a(\a[28] ), .b(\b[27] ), .out0(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n306), .o1(new_n311));
  aoi022aa1n03x5               g216(.a(new_n309), .b(new_n310), .c(new_n305), .d(new_n311), .o1(\s[28] ));
  and002aa1n02x5               g217(.a(new_n310), .b(new_n300), .o(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n304), .c(new_n193), .d(new_n294), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n313), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n307), .carry(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n295), .d(new_n298), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .out0(new_n318));
  norb02aa1n02x5               g223(.a(new_n316), .b(new_n318), .out0(new_n319));
  aoi022aa1n03x5               g224(.a(new_n317), .b(new_n318), .c(new_n314), .d(new_n319), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g226(.a(new_n308), .b(new_n310), .c(new_n318), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n304), .c(new_n193), .d(new_n294), .o1(new_n323));
  inv000aa1n02x5               g228(.a(new_n322), .o1(new_n324));
  inv000aa1d42x5               g229(.a(\b[28] ), .o1(new_n325));
  inv000aa1d42x5               g230(.a(\a[29] ), .o1(new_n326));
  oaib12aa1n02x5               g231(.a(new_n316), .b(\b[28] ), .c(new_n326), .out0(new_n327));
  oaib12aa1n02x5               g232(.a(new_n327), .b(new_n325), .c(\a[29] ), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n295), .d(new_n298), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  oaoi13aa1n04x5               g235(.a(new_n330), .b(new_n327), .c(new_n326), .d(new_n325), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n329), .b(new_n330), .c(new_n323), .d(new_n331), .o1(\s[30] ));
  nanb02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  nanb02aa1n02x5               g238(.a(\b[30] ), .b(\a[31] ), .out0(new_n334));
  nanp02aa1n02x5               g239(.a(new_n334), .b(new_n333), .o1(new_n335));
  nano32aa1n06x5               g240(.a(new_n308), .b(new_n330), .c(new_n310), .d(new_n318), .out0(new_n336));
  aoai13aa1n02x5               g241(.a(new_n336), .b(new_n304), .c(new_n193), .d(new_n294), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n336), .o1(new_n338));
  norp02aa1n02x5               g243(.a(\b[29] ), .b(\a[30] ), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n339), .b(new_n327), .c(new_n340), .o1(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n338), .c(new_n295), .d(new_n298), .o1(new_n342));
  oai112aa1n02x5               g247(.a(new_n333), .b(new_n334), .c(\b[29] ), .d(\a[30] ), .o1(new_n343));
  aoi012aa1n02x5               g248(.a(new_n343), .b(new_n327), .c(new_n340), .o1(new_n344));
  aoi022aa1n03x5               g249(.a(new_n342), .b(new_n335), .c(new_n337), .d(new_n344), .o1(\s[31] ));
  norb02aa1n02x5               g250(.a(new_n107), .b(new_n103), .out0(new_n346));
  xobna2aa1n03x5               g251(.a(new_n346), .b(new_n102), .c(new_n101), .out0(\s[3] ));
  aob012aa1n02x5               g252(.a(new_n346), .b(new_n102), .c(new_n101), .out0(new_n348));
  nanb02aa1n02x5               g253(.a(new_n105), .b(new_n106), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n348), .c(new_n107), .out0(\s[4] ));
  xnbna2aa1n03x5               g255(.a(new_n112), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  norp02aa1n02x5               g256(.a(\b[4] ), .b(\a[5] ), .o1(new_n352));
  aoi012aa1n02x5               g257(.a(new_n352), .b(new_n111), .c(new_n112), .o1(new_n353));
  norp03aa1n02x5               g258(.a(new_n115), .b(new_n114), .c(new_n352), .o1(new_n354));
  aob012aa1n02x5               g259(.a(new_n354), .b(new_n111), .c(new_n112), .out0(new_n355));
  oai012aa1n02x5               g260(.a(new_n355), .b(new_n353), .c(new_n116), .o1(\s[6] ));
  nanb02aa1n02x5               g261(.a(new_n115), .b(new_n355), .out0(new_n357));
  aoi022aa1n02x5               g262(.a(new_n357), .b(new_n139), .c(new_n123), .d(new_n355), .o1(\s[7] ));
  aoi012aa1n02x5               g263(.a(new_n117), .b(new_n355), .c(new_n123), .o1(new_n359));
  aoi112aa1n02x5               g264(.a(new_n113), .b(new_n117), .c(new_n355), .d(new_n123), .o1(new_n360));
  oab012aa1n02x4               g265(.a(new_n360), .b(new_n359), .c(new_n138), .out0(\s[8] ));
  aoi112aa1n02x5               g266(.a(new_n126), .b(new_n127), .c(new_n111), .d(new_n120), .o1(new_n362));
  aoi012aa1n02x5               g267(.a(new_n362), .b(new_n141), .c(new_n127), .o1(\s[9] ));
endmodule


