// Benchmark "adder" written by ABC on Thu Jul 18 12:23:43 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n340, new_n342, new_n343, new_n346, new_n347,
    new_n348, new_n351, new_n352, new_n353, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1n16x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand02aa1d06x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  tech160nm_fioaoi03aa1n04x5   g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor022aa1n08x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1n04x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  tech160nm_fioai012aa1n03p5x5 g015(.a(new_n107), .b(new_n108), .c(new_n106), .o1(new_n111));
  oai012aa1n12x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  norp02aa1n12x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nor002aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  tech160nm_fixnrc02aa1n04x5   g022(.a(\b[6] ), .b(\a[7] ), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[7] ), .b(\a[8] ), .out0(new_n119));
  norp03aa1d12x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  nor002aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  aoi022aa1d24x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  oaoi13aa1n09x5               g027(.a(new_n121), .b(new_n122), .c(new_n113), .d(new_n115), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  nand42aa1n06x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n100), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n112), .d(new_n120), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n99), .b(new_n127), .c(new_n101), .out0(\s[10] ));
  nona22aa1n02x4               g033(.a(new_n127), .b(new_n100), .c(new_n97), .out0(new_n129));
  nand42aa1d28x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norp02aa1n12x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nano22aa1n02x4               g036(.a(new_n131), .b(new_n98), .c(new_n130), .out0(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n130), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n103), .b(new_n102), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[1] ), .b(\a[2] ), .o1(new_n135));
  aob012aa1n02x5               g040(.a(new_n134), .b(new_n104), .c(new_n135), .out0(new_n136));
  norb02aa1n02x5               g041(.a(new_n107), .b(new_n106), .out0(new_n137));
  norb02aa1n02x5               g042(.a(new_n109), .b(new_n108), .out0(new_n138));
  nand23aa1n03x5               g043(.a(new_n136), .b(new_n137), .c(new_n138), .o1(new_n139));
  norb02aa1n02x7               g044(.a(new_n114), .b(new_n113), .out0(new_n140));
  nanb02aa1n02x5               g045(.a(new_n115), .b(new_n116), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n119), .o1(new_n142));
  nona23aa1n02x4               g047(.a(new_n142), .b(new_n140), .c(new_n118), .d(new_n141), .out0(new_n143));
  and002aa1n02x5               g048(.a(\b[7] ), .b(\a[8] ), .o(new_n144));
  norp02aa1n02x5               g049(.a(\b[7] ), .b(\a[8] ), .o1(new_n145));
  oab012aa1n02x4               g050(.a(new_n145), .b(new_n123), .c(new_n144), .out0(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n143), .c(new_n139), .d(new_n111), .o1(new_n147));
  oai022aa1d24x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n98), .b(new_n148), .c(new_n147), .d(new_n126), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n133), .c(new_n129), .d(new_n132), .o1(\s[11] ));
  nor002aa1d32x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nand42aa1d28x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n131), .c(new_n129), .d(new_n132), .o1(new_n154));
  nona22aa1n02x4               g059(.a(new_n152), .b(new_n151), .c(new_n131), .out0(new_n155));
  aoai13aa1n02x5               g060(.a(new_n154), .b(new_n155), .c(new_n132), .d(new_n129), .o1(\s[12] ));
  nano23aa1d15x5               g061(.a(new_n151), .b(new_n131), .c(new_n152), .d(new_n130), .out0(new_n157));
  nano23aa1n06x5               g062(.a(new_n97), .b(new_n100), .c(new_n125), .d(new_n98), .out0(new_n158));
  nand22aa1n12x5               g063(.a(new_n158), .b(new_n157), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n124), .c(new_n112), .d(new_n120), .o1(new_n161));
  nanb03aa1n09x5               g066(.a(new_n151), .b(new_n152), .c(new_n130), .out0(new_n162));
  oai112aa1n06x5               g067(.a(new_n148), .b(new_n98), .c(\b[10] ), .d(\a[11] ), .o1(new_n163));
  tech160nm_fioai012aa1n03p5x5 g068(.a(new_n152), .b(new_n151), .c(new_n131), .o1(new_n164));
  oai012aa1n18x5               g069(.a(new_n164), .b(new_n163), .c(new_n162), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  nor002aa1d32x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nand02aa1n03x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n161), .c(new_n166), .out0(\s[13] ));
  inv000aa1d42x5               g075(.a(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n169), .b(new_n165), .c(new_n147), .d(new_n160), .o1(new_n172));
  nor022aa1n08x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand02aa1n06x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  oaih22aa1d12x5               g080(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n176));
  nanb03aa1n02x5               g081(.a(new_n176), .b(new_n172), .c(new_n174), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n175), .c(new_n171), .d(new_n172), .o1(\s[14] ));
  nano23aa1n06x5               g083(.a(new_n167), .b(new_n173), .c(new_n174), .d(new_n168), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n165), .c(new_n147), .d(new_n160), .o1(new_n180));
  oai012aa1n02x5               g085(.a(new_n174), .b(new_n173), .c(new_n167), .o1(new_n181));
  nor042aa1n12x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand42aa1n08x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(new_n184));
  xobna2aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n181), .out0(\s[15] ));
  nona23aa1n03x5               g090(.a(new_n174), .b(new_n168), .c(new_n167), .d(new_n173), .out0(new_n186));
  aoai13aa1n02x5               g091(.a(new_n181), .b(new_n186), .c(new_n161), .d(new_n166), .o1(new_n187));
  nor042aa1n03x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  tech160nm_finand02aa1n03p5x5 g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nanb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n182), .c(new_n187), .d(new_n183), .o1(new_n191));
  norb03aa1n02x5               g096(.a(new_n189), .b(new_n182), .c(new_n188), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n184), .c(new_n180), .d(new_n181), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n191), .b(new_n193), .o1(\s[16] ));
  nano23aa1n03x7               g099(.a(new_n182), .b(new_n188), .c(new_n189), .d(new_n183), .out0(new_n195));
  nano22aa1d15x5               g100(.a(new_n159), .b(new_n179), .c(new_n195), .out0(new_n196));
  aoai13aa1n12x5               g101(.a(new_n196), .b(new_n124), .c(new_n112), .d(new_n120), .o1(new_n197));
  nor043aa1n06x5               g102(.a(new_n186), .b(new_n184), .c(new_n190), .o1(new_n198));
  aoi013aa1n03x5               g103(.a(new_n182), .b(new_n176), .c(new_n174), .d(new_n183), .o1(new_n199));
  oaoi03aa1n06x5               g104(.a(\a[16] ), .b(\b[15] ), .c(new_n199), .o1(new_n200));
  aoi012aa1d24x5               g105(.a(new_n200), .b(new_n165), .c(new_n198), .o1(new_n201));
  xnrc02aa1n12x5               g106(.a(\b[16] ), .b(\a[17] ), .out0(new_n202));
  xobna2aa1n03x5               g107(.a(new_n202), .b(new_n197), .c(new_n201), .out0(\s[17] ));
  aoi012aa1n02x5               g108(.a(new_n202), .b(new_n197), .c(new_n201), .o1(new_n204));
  nor042aa1n03x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand02aa1n16x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n03x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nand42aa1n10x5               g112(.a(new_n197), .b(new_n201), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  aoib12aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n202), .out0(new_n210));
  nona22aa1n02x4               g115(.a(new_n206), .b(new_n205), .c(new_n209), .out0(new_n211));
  oai022aa1n02x5               g116(.a(new_n210), .b(new_n207), .c(new_n211), .d(new_n204), .o1(\s[18] ));
  norb02aa1n06x5               g117(.a(new_n207), .b(new_n202), .out0(new_n213));
  inv020aa1n08x5               g118(.a(new_n213), .o1(new_n214));
  oai012aa1n02x5               g119(.a(new_n206), .b(new_n205), .c(new_n209), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n214), .c(new_n197), .d(new_n201), .o1(new_n216));
  norp02aa1n02x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nand42aa1n03x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n06x4               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  oaih22aa1n04x5               g124(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n220));
  aoi122aa1n06x5               g125(.a(new_n219), .b(new_n206), .c(new_n220), .d(new_n208), .e(new_n213), .o1(new_n221));
  aoi012aa1n02x5               g126(.a(new_n221), .b(new_n216), .c(new_n219), .o1(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand42aa1n06x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n217), .c(new_n216), .d(new_n218), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\a[19] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[18] ), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n228), .o1(new_n230));
  nanb02aa1n02x5               g135(.a(new_n226), .b(new_n230), .out0(new_n231));
  aoai13aa1n02x5               g136(.a(new_n227), .b(new_n231), .c(new_n219), .d(new_n216), .o1(\s[20] ));
  nano23aa1n06x5               g137(.a(new_n226), .b(new_n202), .c(new_n207), .d(new_n219), .out0(new_n233));
  nanb03aa1n02x5               g138(.a(new_n224), .b(new_n225), .c(new_n218), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n220), .b(new_n230), .c(new_n206), .o1(new_n235));
  aoai13aa1n12x5               g140(.a(new_n225), .b(new_n224), .c(new_n228), .d(new_n229), .o1(new_n236));
  oai012aa1n06x5               g141(.a(new_n236), .b(new_n235), .c(new_n234), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[21] ), .b(\b[20] ), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(new_n208), .d(new_n233), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n238), .b(new_n237), .c(new_n208), .d(new_n233), .o1(new_n240));
  norb02aa1n03x4               g145(.a(new_n239), .b(new_n240), .out0(\s[21] ));
  orn002aa1n02x5               g146(.a(\a[21] ), .b(\b[20] ), .o(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[21] ), .b(\a[22] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  and002aa1n02x5               g149(.a(\b[21] ), .b(\a[22] ), .o(new_n245));
  oai022aa1n02x5               g150(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n246));
  nona22aa1n02x5               g151(.a(new_n239), .b(new_n245), .c(new_n246), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n244), .c(new_n242), .d(new_n239), .o1(\s[22] ));
  nano23aa1n03x7               g153(.a(new_n217), .b(new_n224), .c(new_n225), .d(new_n218), .out0(new_n249));
  nano32aa1n03x7               g154(.a(new_n214), .b(new_n244), .c(new_n249), .d(new_n238), .out0(new_n250));
  nano22aa1n03x5               g155(.a(new_n224), .b(new_n218), .c(new_n225), .out0(new_n251));
  oai012aa1n02x5               g156(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n252));
  oab012aa1n03x5               g157(.a(new_n252), .b(new_n209), .c(new_n205), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n236), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[20] ), .b(\a[21] ), .o1(new_n255));
  nano22aa1n03x7               g160(.a(new_n243), .b(new_n242), .c(new_n255), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n254), .c(new_n253), .d(new_n251), .o1(new_n257));
  oaoi03aa1n12x5               g162(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[23] ), .b(\b[22] ), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n208), .d(new_n250), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n261), .b(new_n260), .c(new_n208), .d(new_n250), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n262), .b(new_n263), .out0(\s[23] ));
  norp02aa1n02x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[24] ), .b(\b[23] ), .out0(new_n267));
  and002aa1n02x5               g172(.a(\b[23] ), .b(\a[24] ), .o(new_n268));
  oai022aa1n02x5               g173(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n269));
  nona22aa1n02x5               g174(.a(new_n262), .b(new_n268), .c(new_n269), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n267), .c(new_n266), .d(new_n262), .o1(\s[24] ));
  nanp02aa1n02x5               g176(.a(new_n267), .b(new_n261), .o1(new_n272));
  nano32aa1n02x4               g177(.a(new_n272), .b(new_n213), .c(new_n256), .d(new_n249), .out0(new_n273));
  aob012aa1n02x5               g178(.a(new_n269), .b(\b[23] ), .c(\a[24] ), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n272), .c(new_n257), .d(new_n259), .o1(new_n275));
  xnrc02aa1n12x5               g180(.a(\b[24] ), .b(\a[25] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n275), .c(new_n208), .d(new_n273), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n277), .b(new_n275), .c(new_n208), .d(new_n273), .o1(new_n279));
  norb02aa1n03x4               g184(.a(new_n278), .b(new_n279), .out0(\s[25] ));
  nor042aa1n02x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  and002aa1n02x5               g188(.a(\b[25] ), .b(\a[26] ), .o(new_n284));
  inv000aa1d42x5               g189(.a(\a[26] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(\b[25] ), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n281), .b(new_n285), .c(new_n286), .o1(new_n287));
  norb02aa1n02x5               g192(.a(new_n287), .b(new_n284), .out0(new_n288));
  tech160nm_finand02aa1n03p5x5 g193(.a(new_n278), .b(new_n288), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n283), .c(new_n282), .d(new_n278), .o1(\s[26] ));
  norb02aa1n09x5               g195(.a(new_n283), .b(new_n276), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  nona22aa1n03x5               g197(.a(new_n250), .b(new_n272), .c(new_n292), .out0(new_n293));
  aoi012aa1n09x5               g198(.a(new_n293), .b(new_n197), .c(new_n201), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n272), .o1(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n258), .c(new_n237), .d(new_n256), .o1(new_n296));
  oaoi03aa1n02x5               g201(.a(new_n285), .b(new_n286), .c(new_n281), .o1(new_n297));
  aoai13aa1n12x5               g202(.a(new_n297), .b(new_n292), .c(new_n296), .d(new_n274), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  nanb02aa1n02x5               g204(.a(new_n299), .b(new_n297), .out0(new_n300));
  aoi112aa1n02x5               g205(.a(new_n294), .b(new_n300), .c(new_n275), .d(new_n291), .o1(new_n301));
  oaoi13aa1n02x5               g206(.a(new_n301), .b(new_n299), .c(new_n294), .d(new_n298), .o1(\s[27] ));
  xorc02aa1n12x5               g207(.a(\a[28] ), .b(\b[27] ), .out0(new_n303));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  oaoi13aa1n06x5               g209(.a(new_n304), .b(new_n299), .c(new_n298), .d(new_n294), .o1(new_n305));
  oaih12aa1n02x5               g210(.a(new_n299), .b(new_n298), .c(new_n294), .o1(new_n306));
  oai112aa1n03x5               g211(.a(new_n306), .b(new_n303), .c(\b[26] ), .d(\a[27] ), .o1(new_n307));
  oaih12aa1n02x5               g212(.a(new_n307), .b(new_n305), .c(new_n303), .o1(\s[28] ));
  and002aa1n02x5               g213(.a(new_n303), .b(new_n299), .o(new_n309));
  oaih12aa1n02x5               g214(.a(new_n309), .b(new_n298), .c(new_n294), .o1(new_n310));
  tech160nm_finand02aa1n03p5x5 g215(.a(new_n198), .b(new_n165), .o1(new_n311));
  nanb02aa1n02x5               g216(.a(new_n200), .b(new_n311), .out0(new_n312));
  nano32aa1n03x7               g217(.a(new_n292), .b(new_n233), .c(new_n256), .d(new_n295), .out0(new_n313));
  aoai13aa1n06x5               g218(.a(new_n313), .b(new_n312), .c(new_n147), .d(new_n196), .o1(new_n314));
  aobi12aa1n03x5               g219(.a(new_n297), .b(new_n275), .c(new_n291), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n309), .o1(new_n316));
  orn002aa1n02x5               g221(.a(\a[27] ), .b(\b[26] ), .o(new_n317));
  oao003aa1n03x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n317), .carry(new_n318));
  aoai13aa1n02x7               g223(.a(new_n318), .b(new_n316), .c(new_n315), .d(new_n314), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n310), .d(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g227(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g228(.a(new_n299), .b(new_n320), .c(new_n303), .o(new_n324));
  oaih12aa1n02x5               g229(.a(new_n324), .b(new_n298), .c(new_n294), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n324), .o1(new_n326));
  oaoi03aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .o1(new_n327));
  inv000aa1n03x5               g232(.a(new_n327), .o1(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n315), .d(new_n314), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  norp02aa1n02x5               g235(.a(new_n327), .b(new_n330), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n329), .b(new_n330), .c(new_n325), .d(new_n331), .o1(\s[30] ));
  nand03aa1n02x5               g237(.a(new_n309), .b(new_n320), .c(new_n330), .o1(new_n333));
  oabi12aa1n02x5               g238(.a(new_n333), .b(new_n298), .c(new_n294), .out0(new_n334));
  xorc02aa1n02x5               g239(.a(\a[31] ), .b(\b[30] ), .out0(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n328), .carry(new_n336));
  norb02aa1n02x5               g241(.a(new_n336), .b(new_n335), .out0(new_n337));
  aoai13aa1n03x5               g242(.a(new_n336), .b(new_n333), .c(new_n315), .d(new_n314), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n334), .d(new_n337), .o1(\s[31] ));
  nanp03aa1n02x5               g244(.a(new_n134), .b(new_n104), .c(new_n135), .o1(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n138), .b(new_n340), .c(new_n134), .out0(\s[3] ));
  inv000aa1d42x5               g246(.a(new_n108), .o1(new_n342));
  nanp02aa1n02x5               g247(.a(new_n136), .b(new_n138), .o1(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n137), .b(new_n343), .c(new_n342), .out0(\s[4] ));
  xnbna2aa1n03x5               g249(.a(new_n140), .b(new_n139), .c(new_n111), .out0(\s[5] ));
  aoai13aa1n02x5               g250(.a(new_n141), .b(new_n113), .c(new_n112), .d(new_n114), .o1(new_n346));
  norb03aa1n02x5               g251(.a(new_n116), .b(new_n113), .c(new_n115), .out0(new_n347));
  aob012aa1n02x5               g252(.a(new_n347), .b(new_n112), .c(new_n140), .out0(new_n348));
  nanp02aa1n02x5               g253(.a(new_n346), .b(new_n348), .o1(\s[6] ));
  xnbna2aa1n03x5               g254(.a(new_n118), .b(new_n348), .c(new_n116), .out0(\s[7] ));
  inv000aa1d42x5               g255(.a(\b[6] ), .o1(new_n351));
  nanb02aa1n02x5               g256(.a(\a[7] ), .b(new_n351), .out0(new_n352));
  nanb03aa1n02x5               g257(.a(new_n118), .b(new_n348), .c(new_n116), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n142), .b(new_n353), .c(new_n352), .out0(\s[8] ));
  aoi112aa1n02x5               g259(.a(new_n126), .b(new_n124), .c(new_n112), .d(new_n120), .o1(new_n355));
  aoi012aa1n02x5               g260(.a(new_n355), .b(new_n147), .c(new_n126), .o1(\s[9] ));
endmodule


