// Benchmark "adder" written by ABC on Thu Jul 18 12:19:22 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n340, new_n342, new_n345, new_n346, new_n347,
    new_n349, new_n350, new_n352, new_n353, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n04x5   g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  orn002aa1n24x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand02aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor002aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n06x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nor002aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n06x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n06x4               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1d12x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  tech160nm_fioai012aa1n03p5x5 g014(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n110));
  nand02aa1d04x5               g015(.a(new_n109), .b(new_n110), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand02aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1d06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor002aa1n03x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n116));
  nor042aa1n06x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand42aa1n06x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  norb02aa1n02x7               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nor042aa1n06x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  nanp02aa1n04x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  norb02aa1n02x5               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  nano22aa1n03x7               g027(.a(new_n116), .b(new_n119), .c(new_n122), .out0(new_n123));
  aoi012aa1n06x5               g028(.a(new_n115), .b(new_n112), .c(new_n114), .o1(new_n124));
  inv000aa1d42x5               g029(.a(\a[8] ), .o1(new_n125));
  inv000aa1d42x5               g030(.a(\b[7] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(new_n126), .b(new_n125), .o1(new_n127));
  inv000aa1n04x5               g032(.a(new_n112), .o1(new_n128));
  tech160nm_fioai012aa1n03p5x5 g033(.a(new_n128), .b(new_n120), .c(new_n117), .o1(new_n129));
  aoi022aa1n06x5               g034(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n130));
  nano32aa1n02x4               g035(.a(new_n129), .b(new_n130), .c(new_n127), .d(new_n113), .out0(new_n131));
  nanb02aa1n06x5               g036(.a(new_n131), .b(new_n124), .out0(new_n132));
  tech160nm_fixorc02aa1n05x5   g037(.a(\a[9] ), .b(\b[8] ), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n132), .c(new_n111), .d(new_n123), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n97), .b(new_n134), .c(new_n98), .out0(\s[10] ));
  oai022aa1n04x5               g040(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n136));
  nanb02aa1n03x5               g041(.a(new_n136), .b(new_n134), .out0(new_n137));
  nor002aa1n16x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi022aa1n06x5               g043(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nand42aa1n06x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n138), .b(new_n141), .out0(new_n142));
  aob012aa1n02x5               g047(.a(new_n137), .b(\b[9] ), .c(\a[10] ), .out0(new_n143));
  aoi022aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n137), .d(new_n140), .o1(\s[11] ));
  inv000aa1d42x5               g049(.a(\a[12] ), .o1(new_n145));
  inv000aa1d42x5               g050(.a(\a[11] ), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[10] ), .o1(new_n147));
  aoi022aa1n03x5               g052(.a(new_n137), .b(new_n139), .c(new_n147), .d(new_n146), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[11] ), .c(new_n145), .out0(\s[12] ));
  nor042aa1n04x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nand02aa1n16x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nano23aa1n02x5               g056(.a(new_n138), .b(new_n150), .c(new_n151), .d(new_n141), .out0(new_n152));
  and003aa1n02x5               g057(.a(new_n152), .b(new_n133), .c(new_n97), .o(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n132), .c(new_n111), .d(new_n123), .o1(new_n154));
  norb03aa1n03x5               g059(.a(new_n151), .b(new_n138), .c(new_n150), .out0(new_n155));
  oa0012aa1n03x5               g060(.a(new_n151), .b(new_n150), .c(new_n138), .o(new_n156));
  aoi013aa1n06x4               g061(.a(new_n156), .b(new_n155), .c(new_n139), .d(new_n136), .o1(new_n157));
  nor042aa1n12x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand02aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n154), .c(new_n157), .out0(\s[13] ));
  nanp02aa1n02x5               g066(.a(new_n154), .b(new_n157), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n158), .b(new_n162), .c(new_n159), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nano23aa1n02x4               g069(.a(new_n115), .b(new_n112), .c(new_n113), .d(new_n114), .out0(new_n165));
  nano23aa1n02x4               g070(.a(new_n117), .b(new_n120), .c(new_n121), .d(new_n118), .out0(new_n166));
  nanp02aa1n02x5               g071(.a(new_n166), .b(new_n165), .o1(new_n167));
  inv000aa1n02x5               g072(.a(new_n124), .o1(new_n168));
  nor042aa1n03x5               g073(.a(new_n131), .b(new_n168), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n167), .c(new_n109), .d(new_n110), .o1(new_n170));
  nanp03aa1n02x5               g075(.a(new_n155), .b(new_n136), .c(new_n139), .o1(new_n171));
  oaib12aa1n02x5               g076(.a(new_n171), .b(new_n155), .c(new_n151), .out0(new_n172));
  nor042aa1n06x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n04x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nano23aa1n06x5               g079(.a(new_n158), .b(new_n173), .c(new_n174), .d(new_n159), .out0(new_n175));
  aoai13aa1n03x5               g080(.a(new_n175), .b(new_n172), .c(new_n170), .d(new_n153), .o1(new_n176));
  oai012aa1n02x5               g081(.a(new_n174), .b(new_n173), .c(new_n158), .o1(new_n177));
  norp02aa1n04x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand42aa1n03x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n06x4               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n176), .c(new_n177), .out0(\s[15] ));
  nand02aa1n03x5               g086(.a(new_n176), .b(new_n177), .o1(new_n182));
  xorc02aa1n12x5               g087(.a(\a[16] ), .b(\b[15] ), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n184), .b(new_n178), .c(new_n182), .d(new_n179), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n180), .o1(new_n186));
  nona22aa1n03x5               g091(.a(new_n186), .b(new_n184), .c(new_n178), .out0(new_n187));
  nanp02aa1n03x5               g092(.a(new_n187), .b(new_n185), .o1(\s[16] ));
  nanp03aa1d12x5               g093(.a(new_n175), .b(new_n180), .c(new_n183), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n189), .b(new_n152), .c(new_n133), .d(new_n97), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n132), .c(new_n111), .d(new_n123), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[15] ), .o1(new_n192));
  inv040aa1d32x5               g097(.a(\a[16] ), .o1(new_n193));
  aoi022aa1n03x5               g098(.a(new_n192), .b(new_n193), .c(\a[15] ), .d(\b[14] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(new_n192), .c(\a[16] ), .out0(new_n195));
  oai122aa1n02x7               g100(.a(new_n174), .b(new_n173), .c(new_n158), .d(\b[14] ), .e(\a[15] ), .o1(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n193), .b(new_n192), .c(new_n178), .o1(new_n197));
  oai012aa1n03x5               g102(.a(new_n197), .b(new_n196), .c(new_n195), .o1(new_n198));
  oabi12aa1n06x5               g103(.a(new_n198), .b(new_n157), .c(new_n189), .out0(new_n199));
  inv040aa1n06x5               g104(.a(new_n199), .o1(new_n200));
  nanp02aa1n06x5               g105(.a(new_n191), .b(new_n200), .o1(new_n201));
  xorc02aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .out0(new_n202));
  aoib12aa1n02x5               g107(.a(new_n189), .b(new_n171), .c(new_n156), .out0(new_n203));
  norp03aa1n02x5               g108(.a(new_n203), .b(new_n198), .c(new_n202), .o1(new_n204));
  aoi022aa1n02x5               g109(.a(new_n201), .b(new_n202), .c(new_n191), .d(new_n204), .o1(\s[17] ));
  inv040aa1d32x5               g110(.a(\a[18] ), .o1(new_n206));
  nor042aa1n09x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  tech160nm_fiaoi012aa1n05x5   g112(.a(new_n207), .b(new_n201), .c(new_n202), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[17] ), .c(new_n206), .out0(\s[18] ));
  inv000aa1d42x5               g114(.a(\a[17] ), .o1(new_n210));
  xroi22aa1d06x4               g115(.a(new_n210), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n199), .c(new_n170), .d(new_n190), .o1(new_n212));
  nand02aa1n06x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nor022aa1n08x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nor002aa1n03x5               g119(.a(new_n214), .b(new_n207), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n213), .b(new_n215), .out0(new_n216));
  inv000aa1n02x5               g121(.a(new_n216), .o1(new_n217));
  nor042aa1n04x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand42aa1n10x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  norb02aa1n03x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n212), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g127(.a(new_n212), .b(new_n217), .o1(new_n223));
  nor002aa1n16x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand02aa1n12x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n218), .c(new_n223), .d(new_n219), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n223), .b(new_n220), .o1(new_n228));
  nona22aa1n02x5               g133(.a(new_n228), .b(new_n226), .c(new_n218), .out0(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n227), .o1(\s[20] ));
  nanb03aa1d18x5               g135(.a(new_n226), .b(new_n211), .c(new_n220), .out0(new_n231));
  nanb03aa1n06x5               g136(.a(new_n224), .b(new_n225), .c(new_n219), .out0(new_n232));
  orn002aa1n03x5               g137(.a(\a[19] ), .b(\b[18] ), .o(new_n233));
  oai112aa1n06x5               g138(.a(new_n233), .b(new_n213), .c(new_n214), .d(new_n207), .o1(new_n234));
  aoi012aa1n12x5               g139(.a(new_n224), .b(new_n218), .c(new_n225), .o1(new_n235));
  oai012aa1n18x5               g140(.a(new_n235), .b(new_n234), .c(new_n232), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n04x5               g142(.a(new_n237), .b(new_n231), .c(new_n191), .d(new_n200), .o1(new_n238));
  tech160nm_fixorc02aa1n05x5   g143(.a(\a[21] ), .b(\b[20] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n231), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n239), .b(new_n236), .c(new_n201), .d(new_n240), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n238), .c(new_n239), .o1(\s[21] ));
  nor042aa1n06x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n243), .c(new_n238), .d(new_n239), .o1(new_n245));
  nand02aa1n02x5               g150(.a(new_n238), .b(new_n239), .o1(new_n246));
  nona22aa1n02x5               g151(.a(new_n246), .b(new_n244), .c(new_n243), .out0(new_n247));
  nanp02aa1n03x5               g152(.a(new_n247), .b(new_n245), .o1(\s[22] ));
  xnrc02aa1n02x5               g153(.a(\b[20] ), .b(\a[21] ), .out0(new_n249));
  nona32aa1n02x4               g154(.a(new_n201), .b(new_n244), .c(new_n249), .d(new_n231), .out0(new_n250));
  nor042aa1n03x5               g155(.a(new_n244), .b(new_n249), .o1(new_n251));
  nanb02aa1n03x5               g156(.a(new_n231), .b(new_n251), .out0(new_n252));
  nano22aa1n03x7               g157(.a(new_n224), .b(new_n219), .c(new_n225), .out0(new_n253));
  tech160nm_fioai012aa1n03p5x5 g158(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .o1(new_n254));
  nona22aa1n09x5               g159(.a(new_n253), .b(new_n215), .c(new_n254), .out0(new_n255));
  nanb02aa1n06x5               g160(.a(new_n244), .b(new_n239), .out0(new_n256));
  inv000aa1d42x5               g161(.a(\a[22] ), .o1(new_n257));
  inv040aa1d32x5               g162(.a(\b[21] ), .o1(new_n258));
  oao003aa1n03x5               g163(.a(new_n257), .b(new_n258), .c(new_n243), .carry(new_n259));
  inv040aa1n03x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n12x5               g165(.a(new_n260), .b(new_n256), .c(new_n255), .d(new_n235), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n252), .c(new_n191), .d(new_n200), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[23] ), .b(\b[22] ), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n264), .b(new_n259), .c(new_n236), .d(new_n251), .o1(new_n265));
  aoi022aa1n02x5               g170(.a(new_n265), .b(new_n250), .c(new_n263), .d(new_n264), .o1(\s[23] ));
  nor002aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  xnrc02aa1n12x5               g172(.a(\b[23] ), .b(\a[24] ), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n267), .c(new_n263), .d(new_n264), .o1(new_n269));
  nand02aa1n02x5               g174(.a(new_n263), .b(new_n264), .o1(new_n270));
  nona22aa1n02x5               g175(.a(new_n270), .b(new_n268), .c(new_n267), .out0(new_n271));
  nanp02aa1n03x5               g176(.a(new_n271), .b(new_n269), .o1(\s[24] ));
  norb02aa1n03x5               g177(.a(new_n264), .b(new_n268), .out0(new_n273));
  nano22aa1n03x7               g178(.a(new_n231), .b(new_n251), .c(new_n273), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n199), .c(new_n170), .d(new_n190), .o1(new_n275));
  oab012aa1n02x4               g180(.a(new_n254), .b(new_n207), .c(new_n214), .out0(new_n276));
  inv000aa1n02x5               g181(.a(new_n235), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n251), .b(new_n277), .c(new_n276), .d(new_n253), .o1(new_n278));
  inv040aa1n02x5               g183(.a(new_n273), .o1(new_n279));
  inv000aa1d42x5               g184(.a(\a[24] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(\b[23] ), .o1(new_n281));
  oao003aa1n02x5               g186(.a(new_n280), .b(new_n281), .c(new_n267), .carry(new_n282));
  inv030aa1n02x5               g187(.a(new_n282), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n279), .c(new_n278), .d(new_n260), .o1(new_n284));
  nanb02aa1n06x5               g189(.a(new_n284), .b(new_n275), .out0(new_n285));
  xorb03aa1n02x5               g190(.a(new_n285), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  tech160nm_fixorc02aa1n05x5   g192(.a(\a[25] ), .b(\b[24] ), .out0(new_n288));
  nor002aa1n02x5               g193(.a(\b[25] ), .b(\a[26] ), .o1(new_n289));
  nand42aa1n03x5               g194(.a(\b[25] ), .b(\a[26] ), .o1(new_n290));
  nanb02aa1n09x5               g195(.a(new_n289), .b(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n287), .c(new_n285), .d(new_n288), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n288), .b(new_n284), .c(new_n201), .d(new_n274), .o1(new_n293));
  nona22aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n287), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[26] ));
  norb02aa1n09x5               g200(.a(new_n288), .b(new_n291), .out0(new_n296));
  nano32aa1d12x5               g201(.a(new_n231), .b(new_n296), .c(new_n251), .d(new_n273), .out0(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n199), .c(new_n170), .d(new_n190), .o1(new_n298));
  oai012aa1n02x5               g203(.a(new_n290), .b(new_n289), .c(new_n287), .o1(new_n299));
  aobi12aa1n06x5               g204(.a(new_n299), .b(new_n284), .c(new_n296), .out0(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  xnbna2aa1n06x5               g206(.a(new_n301), .b(new_n300), .c(new_n298), .out0(\s[27] ));
  aoai13aa1n04x5               g207(.a(new_n296), .b(new_n282), .c(new_n261), .d(new_n273), .o1(new_n303));
  nand43aa1n03x5               g208(.a(new_n298), .b(new_n303), .c(new_n299), .o1(new_n304));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  norp02aa1n02x5               g210(.a(\b[27] ), .b(\a[28] ), .o1(new_n306));
  nand42aa1n03x5               g211(.a(\b[27] ), .b(\a[28] ), .o1(new_n307));
  nanb02aa1n09x5               g212(.a(new_n306), .b(new_n307), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n304), .d(new_n301), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n273), .b(new_n259), .c(new_n236), .d(new_n251), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n296), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n299), .b(new_n311), .c(new_n310), .d(new_n283), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n301), .b(new_n312), .c(new_n201), .d(new_n297), .o1(new_n313));
  nona22aa1n02x5               g218(.a(new_n313), .b(new_n308), .c(new_n305), .out0(new_n314));
  nanp02aa1n03x5               g219(.a(new_n309), .b(new_n314), .o1(\s[28] ));
  norb02aa1d21x5               g220(.a(new_n301), .b(new_n308), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n312), .c(new_n201), .d(new_n297), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  aoi012aa1n02x5               g223(.a(new_n306), .b(new_n305), .c(new_n307), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n318), .c(new_n300), .d(new_n298), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n319), .b(new_n321), .out0(new_n322));
  aoi022aa1n03x5               g227(.a(new_n320), .b(new_n321), .c(new_n317), .d(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g228(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d21x5               g229(.a(new_n308), .b(new_n301), .c(new_n321), .out0(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n312), .c(new_n201), .d(new_n297), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n325), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[29] ), .b(\b[28] ), .c(new_n319), .carry(new_n328));
  aoai13aa1n02x7               g233(.a(new_n328), .b(new_n327), .c(new_n300), .d(new_n298), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  norb02aa1n02x5               g235(.a(new_n328), .b(new_n330), .out0(new_n331));
  aoi022aa1n03x5               g236(.a(new_n329), .b(new_n330), .c(new_n326), .d(new_n331), .o1(\s[30] ));
  nanp03aa1n02x5               g237(.a(new_n316), .b(new_n321), .c(new_n330), .o1(new_n333));
  nanb02aa1n03x5               g238(.a(new_n333), .b(new_n304), .out0(new_n334));
  xorc02aa1n02x5               g239(.a(\a[31] ), .b(\b[30] ), .out0(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n328), .carry(new_n336));
  norb02aa1n02x5               g241(.a(new_n336), .b(new_n335), .out0(new_n337));
  aoai13aa1n02x7               g242(.a(new_n336), .b(new_n333), .c(new_n300), .d(new_n298), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n334), .d(new_n337), .o1(\s[31] ));
  nanp03aa1n02x5               g244(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n108), .b(new_n340), .c(new_n99), .out0(\s[3] ));
  aoi012aa1n02x5               g246(.a(new_n106), .b(new_n102), .c(new_n107), .o1(new_n342));
  xnrc02aa1n02x5               g247(.a(new_n342), .b(new_n105), .out0(\s[4] ));
  xnbna2aa1n03x5               g248(.a(new_n119), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  aoi012aa1n02x5               g249(.a(new_n117), .b(new_n111), .c(new_n118), .o1(new_n345));
  norb03aa1n02x5               g250(.a(new_n121), .b(new_n117), .c(new_n120), .out0(new_n346));
  aob012aa1n02x5               g251(.a(new_n346), .b(new_n111), .c(new_n119), .out0(new_n347));
  oai012aa1n02x5               g252(.a(new_n347), .b(new_n345), .c(new_n122), .o1(\s[6] ));
  aoi022aa1n02x5               g253(.a(new_n347), .b(new_n121), .c(new_n128), .d(new_n113), .o1(new_n349));
  aoi012aa1n02x5               g254(.a(new_n112), .b(\a[6] ), .c(\b[5] ), .o1(new_n350));
  aoi013aa1n02x4               g255(.a(new_n349), .b(new_n347), .c(new_n113), .d(new_n350), .o1(\s[7] ));
  norb02aa1n02x5               g256(.a(new_n114), .b(new_n115), .out0(new_n352));
  nanp03aa1n02x5               g257(.a(new_n347), .b(new_n113), .c(new_n350), .o1(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n352), .b(new_n353), .c(new_n128), .out0(\s[8] ));
  aoi112aa1n02x5               g259(.a(new_n132), .b(new_n133), .c(new_n111), .d(new_n123), .o1(new_n355));
  aoi012aa1n02x5               g260(.a(new_n355), .b(new_n170), .c(new_n133), .o1(\s[9] ));
endmodule


