// Benchmark "adder" written by ABC on Thu Jul 18 12:24:57 2024

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
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n345, new_n347, new_n348, new_n351, new_n352, new_n353, new_n356,
    new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n20x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n24x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv040aa1d32x5               g006(.a(\a[2] ), .o1(new_n102));
  inv030aa1d32x5               g007(.a(\b[1] ), .o1(new_n103));
  nand22aa1n09x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  tech160nm_fioaoi03aa1n02p5x5 g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nanp02aa1n06x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n06x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  oai012aa1n04x7               g015(.a(new_n107), .b(new_n108), .c(new_n106), .o1(new_n111));
  oai012aa1n12x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand02aa1d06x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  norp02aa1n06x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  tech160nm_fixnrc02aa1n05x5   g022(.a(\b[6] ), .b(\a[7] ), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[7] ), .b(\a[8] ), .out0(new_n119));
  norp03aa1d24x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  orn002aa1n24x5               g025(.a(\a[7] ), .b(\b[6] ), .o(new_n121));
  oai022aa1d18x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  aoi022aa1d24x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n123));
  aobi12aa1n18x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .out0(new_n124));
  oaoi03aa1n12x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  nand42aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n100), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n125), .c(new_n112), .d(new_n120), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n99), .b(new_n128), .c(new_n101), .out0(\s[10] ));
  nor002aa1n04x5               g034(.a(new_n100), .b(new_n97), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n128), .b(new_n130), .o1(new_n131));
  nand42aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor002aa1n08x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nano22aa1n02x4               g038(.a(new_n133), .b(new_n98), .c(new_n132), .out0(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n132), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n131), .b(new_n98), .o1(new_n136));
  aoi022aa1n02x5               g041(.a(new_n136), .b(new_n135), .c(new_n131), .d(new_n134), .o1(\s[11] ));
  nor042aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n141));
  norb03aa1d15x5               g046(.a(new_n139), .b(new_n133), .c(new_n138), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n141), .b(new_n143), .c(new_n134), .d(new_n131), .o1(\s[12] ));
  nano23aa1n06x5               g049(.a(new_n140), .b(new_n135), .c(new_n127), .d(new_n99), .out0(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n125), .c(new_n112), .d(new_n120), .o1(new_n146));
  tech160nm_fioai012aa1n03p5x5 g051(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n147));
  tech160nm_fioai012aa1n05x5   g052(.a(new_n132), .b(\b[11] ), .c(\a[12] ), .o1(new_n148));
  oai013aa1d12x5               g053(.a(new_n142), .b(new_n130), .c(new_n147), .d(new_n148), .o1(new_n149));
  nanp02aa1n06x5               g054(.a(new_n149), .b(new_n139), .o1(new_n150));
  nanp02aa1n03x5               g055(.a(new_n146), .b(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp03aa1n02x5               g060(.a(new_n151), .b(new_n154), .c(new_n155), .o1(new_n156));
  nor002aa1n06x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand42aa1d28x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  oaih22aa1d12x5               g064(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n160));
  nanb03aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n158), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n154), .d(new_n156), .o1(\s[14] ));
  nano23aa1n06x5               g067(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n155), .out0(new_n163));
  nanp02aa1n03x5               g068(.a(new_n151), .b(new_n163), .o1(new_n164));
  oai012aa1n02x5               g069(.a(new_n158), .b(new_n157), .c(new_n153), .o1(new_n165));
  nor042aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1n08x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanb02aa1n03x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  xobna2aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n165), .out0(\s[15] ));
  aoi012aa1n02x5               g074(.a(new_n168), .b(new_n164), .c(new_n165), .o1(new_n170));
  nona23aa1n09x5               g075(.a(new_n158), .b(new_n155), .c(new_n153), .d(new_n157), .out0(new_n171));
  aoai13aa1n02x5               g076(.a(new_n165), .b(new_n171), .c(new_n146), .d(new_n150), .o1(new_n172));
  nor002aa1n16x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  tech160nm_finand02aa1n05x5   g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n03x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n166), .c(new_n172), .d(new_n167), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n173), .o1(new_n177));
  oai112aa1n02x5               g082(.a(new_n177), .b(new_n174), .c(\b[14] ), .d(\a[15] ), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n176), .b(new_n170), .c(new_n178), .o1(\s[16] ));
  nano23aa1n06x5               g084(.a(new_n138), .b(new_n133), .c(new_n139), .d(new_n132), .out0(new_n180));
  nano23aa1n03x7               g085(.a(new_n97), .b(new_n100), .c(new_n126), .d(new_n98), .out0(new_n181));
  nano23aa1n06x5               g086(.a(new_n166), .b(new_n173), .c(new_n174), .d(new_n167), .out0(new_n182));
  nanp02aa1n04x5               g087(.a(new_n182), .b(new_n163), .o1(new_n183));
  nano22aa1n12x5               g088(.a(new_n183), .b(new_n180), .c(new_n181), .out0(new_n184));
  aoai13aa1n12x5               g089(.a(new_n184), .b(new_n125), .c(new_n112), .d(new_n120), .o1(new_n185));
  nor043aa1n03x5               g090(.a(new_n171), .b(new_n168), .c(new_n175), .o1(new_n186));
  aoi013aa1n02x4               g091(.a(new_n166), .b(new_n160), .c(new_n158), .d(new_n167), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(\a[16] ), .b(\b[15] ), .c(new_n187), .o1(new_n188));
  aoi013aa1n09x5               g093(.a(new_n188), .b(new_n186), .c(new_n149), .d(new_n139), .o1(new_n189));
  xorc02aa1n12x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n190), .b(new_n185), .c(new_n189), .out0(\s[17] ));
  xorc02aa1n12x5               g096(.a(\a[18] ), .b(\b[17] ), .out0(new_n192));
  nanp02aa1n09x5               g097(.a(new_n185), .b(new_n189), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n190), .o1(new_n195));
  oai022aa1d24x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  aoi122aa1n06x5               g101(.a(new_n196), .b(\b[17] ), .c(\a[18] ), .d(new_n193), .e(new_n190), .o1(new_n197));
  oabi12aa1n02x5               g102(.a(new_n197), .b(new_n192), .c(new_n195), .out0(\s[18] ));
  nanp02aa1n02x5               g103(.a(new_n103), .b(new_n102), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[1] ), .b(\a[2] ), .o1(new_n200));
  aob012aa1n02x5               g105(.a(new_n199), .b(new_n104), .c(new_n200), .out0(new_n201));
  norb02aa1n02x5               g106(.a(new_n107), .b(new_n106), .out0(new_n202));
  norb02aa1n02x7               g107(.a(new_n109), .b(new_n108), .out0(new_n203));
  nand23aa1n03x5               g108(.a(new_n201), .b(new_n202), .c(new_n203), .o1(new_n204));
  nanb02aa1n09x5               g109(.a(new_n113), .b(new_n114), .out0(new_n205));
  inv020aa1n02x5               g110(.a(new_n205), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n115), .b(new_n116), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n119), .o1(new_n208));
  nona23aa1n02x5               g113(.a(new_n208), .b(new_n206), .c(new_n118), .d(new_n207), .out0(new_n209));
  norp02aa1n02x5               g114(.a(\b[7] ), .b(\a[8] ), .o1(new_n210));
  oai012aa1n02x5               g115(.a(new_n123), .b(new_n115), .c(new_n113), .o1(new_n211));
  aoi022aa1n02x5               g116(.a(new_n211), .b(new_n121), .c(\a[8] ), .d(\b[7] ), .o1(new_n212));
  norp02aa1n02x5               g117(.a(new_n212), .b(new_n210), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n209), .c(new_n204), .d(new_n111), .o1(new_n214));
  oabi12aa1n06x5               g119(.a(new_n188), .b(new_n150), .c(new_n183), .out0(new_n215));
  and002aa1n03x5               g120(.a(new_n192), .b(new_n190), .o(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n215), .c(new_n214), .d(new_n184), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(\b[17] ), .b(\a[18] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n196), .b(new_n218), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[19] ), .b(\b[18] ), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n217), .c(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x7               g127(.a(new_n220), .b(new_n217), .c(new_n219), .out0(new_n223));
  nand42aa1n03x5               g128(.a(new_n217), .b(new_n219), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[19] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[18] ), .o1(new_n226));
  nand02aa1d28x5               g131(.a(new_n226), .b(new_n225), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  nand02aa1n06x5               g133(.a(\b[18] ), .b(\a[19] ), .o1(new_n229));
  nor002aa1n12x5               g134(.a(\b[19] ), .b(\a[20] ), .o1(new_n230));
  nand02aa1d10x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nanb02aa1n06x5               g136(.a(new_n230), .b(new_n231), .out0(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n228), .c(new_n224), .d(new_n229), .o1(new_n233));
  nano22aa1n02x4               g138(.a(new_n230), .b(new_n227), .c(new_n231), .out0(new_n234));
  oaib12aa1n02x5               g139(.a(new_n233), .b(new_n223), .c(new_n234), .out0(\s[20] ));
  nano22aa1n09x5               g140(.a(new_n232), .b(new_n227), .c(new_n229), .out0(new_n236));
  nand23aa1d12x5               g141(.a(new_n236), .b(new_n190), .c(new_n192), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  nanb03aa1n03x5               g143(.a(new_n230), .b(new_n231), .c(new_n229), .out0(new_n239));
  nanp03aa1n03x5               g144(.a(new_n196), .b(new_n227), .c(new_n218), .o1(new_n240));
  aoai13aa1n12x5               g145(.a(new_n231), .b(new_n230), .c(new_n225), .d(new_n226), .o1(new_n241));
  oai012aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .o1(new_n242));
  nor042aa1n06x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nand42aa1n02x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n193), .d(new_n238), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n242), .c(new_n193), .d(new_n238), .o1(new_n247));
  norb02aa1n03x4               g152(.a(new_n246), .b(new_n247), .out0(\s[21] ));
  inv030aa1n03x5               g153(.a(new_n243), .o1(new_n249));
  nor002aa1n02x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  nanp02aa1n04x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  oai022aa1n02x5               g157(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n253));
  nanb03aa1n03x5               g158(.a(new_n253), .b(new_n246), .c(new_n251), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n249), .d(new_n246), .o1(\s[22] ));
  nano23aa1n06x5               g160(.a(new_n243), .b(new_n250), .c(new_n251), .d(new_n244), .out0(new_n256));
  nand23aa1n03x5               g161(.a(new_n216), .b(new_n236), .c(new_n256), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n185), .c(new_n189), .o1(new_n258));
  aoi022aa1n02x5               g163(.a(new_n242), .b(new_n256), .c(new_n251), .d(new_n253), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n257), .c(new_n185), .d(new_n189), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[23] ), .b(\b[22] ), .out0(new_n261));
  oaoi03aa1n02x5               g166(.a(\a[22] ), .b(\b[21] ), .c(new_n249), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n261), .b(new_n262), .c(new_n242), .d(new_n256), .o1(new_n263));
  aboi22aa1n03x5               g168(.a(new_n258), .b(new_n263), .c(new_n260), .d(new_n261), .out0(\s[23] ));
  xorc02aa1n02x5               g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  norp02aa1n02x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n260), .c(new_n261), .o1(new_n267));
  inv000aa1d42x5               g172(.a(\a[24] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(\b[23] ), .o1(new_n269));
  nand22aa1n02x5               g174(.a(new_n260), .b(new_n261), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n266), .b(new_n268), .c(new_n269), .o1(new_n271));
  oai112aa1n02x5               g176(.a(new_n270), .b(new_n271), .c(new_n269), .d(new_n268), .o1(new_n272));
  oai012aa1n02x5               g177(.a(new_n272), .b(new_n267), .c(new_n265), .o1(\s[24] ));
  nano32aa1n03x7               g178(.a(new_n237), .b(new_n265), .c(new_n256), .d(new_n261), .out0(new_n274));
  nano22aa1n02x4               g179(.a(new_n230), .b(new_n229), .c(new_n231), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .o1(new_n276));
  norb02aa1n02x5               g181(.a(new_n196), .b(new_n276), .out0(new_n277));
  inv000aa1n02x5               g182(.a(new_n241), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n256), .b(new_n278), .c(new_n277), .d(new_n275), .o1(new_n279));
  inv000aa1n02x5               g184(.a(new_n262), .o1(new_n280));
  and002aa1n02x5               g185(.a(new_n265), .b(new_n261), .o(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  oaoi03aa1n02x5               g187(.a(new_n268), .b(new_n269), .c(new_n266), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n282), .c(new_n279), .d(new_n280), .o1(new_n284));
  xnrc02aa1n12x5               g189(.a(\b[24] ), .b(\a[25] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n284), .c(new_n193), .d(new_n274), .o1(new_n287));
  aoi112aa1n02x5               g192(.a(new_n286), .b(new_n284), .c(new_n193), .d(new_n274), .o1(new_n288));
  norb02aa1n02x5               g193(.a(new_n287), .b(new_n288), .out0(\s[25] ));
  nor042aa1n03x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[26] ), .b(\b[25] ), .out0(new_n292));
  nanp02aa1n02x5               g197(.a(\b[25] ), .b(\a[26] ), .o1(new_n293));
  inv000aa1d42x5               g198(.a(\a[26] ), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\b[25] ), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n290), .b(new_n294), .c(new_n295), .o1(new_n296));
  nanp03aa1n03x5               g201(.a(new_n287), .b(new_n293), .c(new_n296), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n292), .c(new_n291), .d(new_n287), .o1(\s[26] ));
  norb02aa1n09x5               g203(.a(new_n292), .b(new_n285), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n257), .b(new_n281), .c(new_n299), .out0(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n215), .c(new_n214), .d(new_n184), .o1(new_n301));
  nanp02aa1n02x5               g206(.a(new_n274), .b(new_n299), .o1(new_n302));
  aoi012aa1n06x5               g207(.a(new_n302), .b(new_n185), .c(new_n189), .o1(new_n303));
  aoai13aa1n04x5               g208(.a(new_n281), .b(new_n262), .c(new_n242), .d(new_n256), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n299), .o1(new_n305));
  oaoi03aa1n02x5               g210(.a(new_n294), .b(new_n295), .c(new_n290), .o1(new_n306));
  aoai13aa1n09x5               g211(.a(new_n306), .b(new_n305), .c(new_n304), .d(new_n283), .o1(new_n307));
  xorc02aa1n12x5               g212(.a(\a[27] ), .b(\b[26] ), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n307), .c(new_n303), .o1(new_n309));
  inv000aa1n02x5               g214(.a(new_n296), .o1(new_n310));
  aoi122aa1n02x5               g215(.a(new_n308), .b(new_n293), .c(new_n310), .d(new_n284), .e(new_n299), .o1(new_n311));
  aobi12aa1n02x7               g216(.a(new_n309), .b(new_n311), .c(new_n301), .out0(\s[27] ));
  xorc02aa1n12x5               g217(.a(\a[28] ), .b(\b[27] ), .out0(new_n313));
  norp02aa1n02x5               g218(.a(\b[26] ), .b(\a[27] ), .o1(new_n314));
  oaoi13aa1n03x5               g219(.a(new_n314), .b(new_n308), .c(new_n307), .d(new_n303), .o1(new_n315));
  oai112aa1n03x5               g220(.a(new_n309), .b(new_n313), .c(\b[26] ), .d(\a[27] ), .o1(new_n316));
  oai012aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n313), .o1(\s[28] ));
  and002aa1n02x5               g222(.a(new_n313), .b(new_n308), .o(new_n318));
  oaih12aa1n02x5               g223(.a(new_n318), .b(new_n307), .c(new_n303), .o1(new_n319));
  orn002aa1n02x5               g224(.a(\a[27] ), .b(\b[26] ), .o(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n320), .carry(new_n321));
  nand42aa1n02x5               g226(.a(new_n319), .b(new_n321), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n321), .b(new_n323), .out0(new_n324));
  aoi022aa1n02x7               g229(.a(new_n322), .b(new_n323), .c(new_n319), .d(new_n324), .o1(\s[29] ));
  xorb03aa1n02x5               g230(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g231(.a(new_n308), .b(new_n323), .c(new_n313), .o(new_n327));
  oaih12aa1n02x5               g232(.a(new_n327), .b(new_n307), .c(new_n303), .o1(new_n328));
  oaoi03aa1n09x5               g233(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n329), .o1(new_n330));
  nand42aa1n02x5               g235(.a(new_n328), .b(new_n330), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .out0(new_n332));
  norp02aa1n02x5               g237(.a(new_n329), .b(new_n332), .o1(new_n333));
  aoi022aa1n02x7               g238(.a(new_n331), .b(new_n332), .c(new_n328), .d(new_n333), .o1(\s[30] ));
  xnrc02aa1n02x5               g239(.a(\b[30] ), .b(\a[31] ), .out0(new_n335));
  and003aa1n06x5               g240(.a(new_n318), .b(new_n332), .c(new_n323), .o(new_n336));
  oaih12aa1n02x5               g241(.a(new_n336), .b(new_n307), .c(new_n303), .o1(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n330), .carry(new_n338));
  tech160nm_fiaoi012aa1n05x5   g243(.a(new_n335), .b(new_n337), .c(new_n338), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(new_n284), .b(new_n299), .c(new_n293), .d(new_n310), .o1(new_n340));
  inv000aa1d42x5               g245(.a(new_n336), .o1(new_n341));
  aoi012aa1n06x5               g246(.a(new_n341), .b(new_n340), .c(new_n301), .o1(new_n342));
  nano22aa1n02x4               g247(.a(new_n342), .b(new_n335), .c(new_n338), .out0(new_n343));
  nor002aa1n02x5               g248(.a(new_n339), .b(new_n343), .o1(\s[31] ));
  nanp03aa1n02x5               g249(.a(new_n199), .b(new_n104), .c(new_n200), .o1(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n203), .b(new_n345), .c(new_n199), .out0(\s[3] ));
  inv000aa1d42x5               g251(.a(new_n108), .o1(new_n347));
  nanp02aa1n02x5               g252(.a(new_n201), .b(new_n203), .o1(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n202), .b(new_n348), .c(new_n347), .out0(\s[4] ));
  xnbna2aa1n03x5               g254(.a(new_n206), .b(new_n204), .c(new_n111), .out0(\s[5] ));
  aoai13aa1n02x5               g255(.a(new_n207), .b(new_n113), .c(new_n112), .d(new_n114), .o1(new_n351));
  norb03aa1n02x5               g256(.a(new_n116), .b(new_n113), .c(new_n115), .out0(new_n352));
  aoai13aa1n06x5               g257(.a(new_n352), .b(new_n205), .c(new_n204), .d(new_n111), .o1(new_n353));
  nanp02aa1n02x5               g258(.a(new_n351), .b(new_n353), .o1(\s[6] ));
  xnbna2aa1n03x5               g259(.a(new_n118), .b(new_n353), .c(new_n116), .out0(\s[7] ));
  nanb03aa1n02x5               g260(.a(new_n118), .b(new_n353), .c(new_n116), .out0(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n208), .b(new_n356), .c(new_n121), .out0(\s[8] ));
  aoi112aa1n02x5               g262(.a(new_n127), .b(new_n125), .c(new_n112), .d(new_n120), .o1(new_n358));
  aoi012aa1n02x5               g263(.a(new_n358), .b(new_n214), .c(new_n127), .o1(\s[9] ));
endmodule


