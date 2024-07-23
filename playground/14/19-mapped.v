// Benchmark "adder" written by ABC on Wed Jul 17 19:17:08 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n159, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n209,
    new_n210, new_n211, new_n212, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n303, new_n304, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n333,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n355, new_n356, new_n358, new_n360, new_n361,
    new_n363, new_n364, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanb02aa1n03x5               g005(.a(new_n99), .b(new_n100), .out0(new_n101));
  orn002aa1n02x7               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nand42aa1n04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n03x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  nand02aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor042aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb03aa1n03x5               g011(.a(new_n105), .b(new_n99), .c(new_n106), .out0(new_n107));
  aoai13aa1n06x5               g012(.a(new_n107), .b(new_n101), .c(new_n104), .d(new_n102), .o1(new_n108));
  nand02aa1n16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  aoi022aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n110));
  oai112aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(\b[7] ), .d(\a[8] ), .o1(new_n111));
  tech160nm_fioai012aa1n04x5   g016(.a(new_n105), .b(\b[6] ), .c(\a[7] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[4] ), .o1(new_n114));
  nanb02aa1n12x5               g019(.a(\a[5] ), .b(new_n114), .out0(new_n115));
  oai112aa1n02x5               g020(.a(new_n115), .b(new_n113), .c(\b[5] ), .d(\a[6] ), .o1(new_n116));
  nor043aa1n02x5               g021(.a(new_n111), .b(new_n116), .c(new_n112), .o1(new_n117));
  nand42aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n06x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand23aa1n04x5               g024(.a(new_n118), .b(new_n109), .c(new_n119), .o1(new_n120));
  oai022aa1n04x5               g025(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n121));
  nand42aa1n02x5               g026(.a(new_n121), .b(new_n118), .o1(new_n122));
  nor002aa1n03x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nor042aa1n03x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  norb03aa1n03x5               g029(.a(new_n109), .b(new_n124), .c(new_n123), .out0(new_n125));
  oai013aa1n03x5               g030(.a(new_n122), .b(new_n125), .c(new_n120), .d(new_n121), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n117), .d(new_n108), .o1(new_n129));
  nor002aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1n08x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  norb02aa1n06x4               g038(.a(new_n100), .b(new_n99), .out0(new_n134));
  nand02aa1n02x5               g039(.a(new_n104), .b(new_n102), .o1(new_n135));
  aobi12aa1n06x5               g040(.a(new_n107), .b(new_n135), .c(new_n134), .out0(new_n136));
  norp02aa1n02x5               g041(.a(\b[7] ), .b(\a[8] ), .o1(new_n137));
  inv040aa1n03x5               g042(.a(new_n120), .o1(new_n138));
  norb03aa1n03x5               g043(.a(new_n113), .b(new_n123), .c(new_n124), .out0(new_n139));
  nona23aa1n03x5               g044(.a(new_n138), .b(new_n139), .c(new_n112), .d(new_n137), .out0(new_n140));
  nor003aa1n03x5               g045(.a(new_n125), .b(new_n121), .c(new_n120), .o1(new_n141));
  norb02aa1n03x5               g046(.a(new_n122), .b(new_n141), .out0(new_n142));
  oai012aa1n12x5               g047(.a(new_n142), .b(new_n136), .c(new_n140), .o1(new_n143));
  aoai13aa1n03x5               g048(.a(new_n132), .b(new_n97), .c(new_n143), .d(new_n127), .o1(new_n144));
  oai012aa1d24x5               g049(.a(new_n131), .b(new_n130), .c(new_n97), .o1(new_n145));
  nor042aa1n09x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand42aa1n02x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norb02aa1n09x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n145), .out0(\s[11] ));
  inv000aa1n02x5               g054(.a(new_n146), .o1(new_n150));
  nanp02aa1n03x5               g055(.a(new_n129), .b(new_n98), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n145), .o1(new_n152));
  aoai13aa1n03x5               g057(.a(new_n148), .b(new_n152), .c(new_n151), .d(new_n132), .o1(new_n153));
  nor042aa1n04x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nanp02aa1n04x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  tech160nm_fiaoi012aa1n02p5x5 g061(.a(new_n156), .b(new_n153), .c(new_n150), .o1(new_n157));
  norb02aa1n03x5               g062(.a(new_n155), .b(new_n154), .out0(new_n158));
  nona22aa1n02x4               g063(.a(new_n153), .b(new_n158), .c(new_n146), .out0(new_n159));
  norb02aa1n03x4               g064(.a(new_n159), .b(new_n157), .out0(\s[12] ));
  aoi012aa1n12x5               g065(.a(new_n126), .b(new_n117), .c(new_n108), .o1(new_n161));
  norb03aa1n02x7               g066(.a(new_n131), .b(new_n97), .c(new_n130), .out0(new_n162));
  nano32aa1n03x7               g067(.a(new_n156), .b(new_n150), .c(new_n147), .d(new_n127), .out0(new_n163));
  nanp02aa1n02x5               g068(.a(new_n163), .b(new_n162), .o1(new_n164));
  tech160nm_fioai012aa1n03p5x5 g069(.a(new_n155), .b(new_n154), .c(new_n146), .o1(new_n165));
  nona23aa1n09x5               g070(.a(new_n155), .b(new_n147), .c(new_n146), .d(new_n154), .out0(new_n166));
  oai012aa1n18x5               g071(.a(new_n165), .b(new_n166), .c(new_n145), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  tech160nm_fioai012aa1n05x5   g073(.a(new_n168), .b(new_n161), .c(new_n164), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g075(.a(\a[14] ), .o1(new_n171));
  nor042aa1d18x5               g076(.a(\b[12] ), .b(\a[13] ), .o1(new_n172));
  nanp02aa1n12x5               g077(.a(\b[12] ), .b(\a[13] ), .o1(new_n173));
  aoi012aa1n02x5               g078(.a(new_n172), .b(new_n169), .c(new_n173), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[13] ), .c(new_n171), .out0(\s[14] ));
  inv000aa1n02x5               g080(.a(new_n164), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n126), .c(new_n108), .d(new_n117), .o1(new_n177));
  nor042aa1n09x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  nand42aa1n20x5               g083(.a(\b[13] ), .b(\a[14] ), .o1(new_n179));
  oaih12aa1n12x5               g084(.a(new_n179), .b(new_n178), .c(new_n172), .o1(new_n180));
  nano23aa1d15x5               g085(.a(new_n172), .b(new_n178), .c(new_n179), .d(new_n173), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  aoai13aa1n04x5               g087(.a(new_n180), .b(new_n182), .c(new_n177), .d(new_n168), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g089(.a(\b[14] ), .b(\a[15] ), .o1(new_n185));
  inv000aa1n02x5               g090(.a(new_n185), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n180), .o1(new_n187));
  tech160nm_finand02aa1n03p5x5 g092(.a(\b[14] ), .b(\a[15] ), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n188), .b(new_n185), .out0(new_n189));
  aoai13aa1n03x5               g094(.a(new_n189), .b(new_n187), .c(new_n169), .d(new_n181), .o1(new_n190));
  norp02aa1n04x5               g095(.a(\b[15] ), .b(\a[16] ), .o1(new_n191));
  nand42aa1n03x5               g096(.a(\b[15] ), .b(\a[16] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  tech160nm_fiaoi012aa1n03p5x5 g099(.a(new_n194), .b(new_n190), .c(new_n186), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n185), .b(new_n193), .c(new_n183), .d(new_n188), .o1(new_n196));
  nor002aa1n02x5               g101(.a(new_n195), .b(new_n196), .o1(\s[16] ));
  nanb02aa1n02x5               g102(.a(\b[13] ), .b(new_n171), .out0(new_n198));
  oai112aa1n04x5               g103(.a(new_n198), .b(new_n179), .c(\b[12] ), .d(\a[13] ), .o1(new_n199));
  nanb03aa1n06x5               g104(.a(new_n191), .b(new_n192), .c(new_n188), .out0(new_n200));
  nano23aa1n06x5               g105(.a(new_n199), .b(new_n200), .c(new_n186), .d(new_n173), .out0(new_n201));
  nanp03aa1n02x5               g106(.a(new_n201), .b(new_n162), .c(new_n163), .o1(new_n202));
  oaoi03aa1n03x5               g107(.a(\a[16] ), .b(\b[15] ), .c(new_n186), .o1(new_n203));
  nona23aa1n02x5               g108(.a(new_n192), .b(new_n188), .c(new_n185), .d(new_n191), .out0(new_n204));
  oabi12aa1n02x7               g109(.a(new_n203), .b(new_n204), .c(new_n180), .out0(new_n205));
  tech160nm_fiaoi012aa1n05x5   g110(.a(new_n205), .b(new_n167), .c(new_n201), .o1(new_n206));
  oai012aa1n12x5               g111(.a(new_n206), .b(new_n161), .c(new_n202), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g113(.a(\a[18] ), .o1(new_n209));
  inv040aa1d28x5               g114(.a(\a[17] ), .o1(new_n210));
  inv040aa1d32x5               g115(.a(\b[16] ), .o1(new_n211));
  oaoi03aa1n03x5               g116(.a(new_n210), .b(new_n211), .c(new_n207), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[17] ), .c(new_n209), .out0(\s[18] ));
  nona22aa1n03x5               g118(.a(new_n181), .b(new_n200), .c(new_n185), .out0(new_n214));
  nano22aa1n03x7               g119(.a(new_n214), .b(new_n163), .c(new_n162), .out0(new_n215));
  nanb03aa1n06x5               g120(.a(new_n145), .b(new_n158), .c(new_n148), .out0(new_n216));
  oab012aa1n06x5               g121(.a(new_n203), .b(new_n204), .c(new_n180), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n214), .c(new_n216), .d(new_n165), .o1(new_n218));
  xroi22aa1d06x4               g123(.a(new_n210), .b(\b[16] ), .c(new_n209), .d(\b[17] ), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n218), .c(new_n143), .d(new_n215), .o1(new_n220));
  nor022aa1n16x5               g125(.a(\b[17] ), .b(\a[18] ), .o1(new_n221));
  nand42aa1n16x5               g126(.a(\b[17] ), .b(\a[18] ), .o1(new_n222));
  aoai13aa1n12x5               g127(.a(new_n222), .b(new_n221), .c(new_n210), .d(new_n211), .o1(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[18] ), .b(\a[19] ), .out0(new_n224));
  inv030aa1n02x5               g129(.a(new_n224), .o1(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n220), .c(new_n223), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g132(.a(\b[18] ), .b(\a[19] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n223), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n225), .b(new_n230), .c(new_n207), .d(new_n219), .o1(new_n231));
  inv040aa1d32x5               g136(.a(\a[20] ), .o1(new_n232));
  inv040aa1d28x5               g137(.a(\b[19] ), .o1(new_n233));
  nand02aa1n02x5               g138(.a(new_n233), .b(new_n232), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[19] ), .b(\a[20] ), .o1(new_n235));
  nand22aa1n06x5               g140(.a(new_n234), .b(new_n235), .o1(new_n236));
  aoi012aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n229), .o1(new_n237));
  tech160nm_fiaoi012aa1n03p5x5 g142(.a(new_n224), .b(new_n220), .c(new_n223), .o1(new_n238));
  nano22aa1n03x7               g143(.a(new_n238), .b(new_n229), .c(new_n236), .out0(new_n239));
  norp02aa1n03x5               g144(.a(new_n237), .b(new_n239), .o1(\s[20] ));
  and002aa1n02x5               g145(.a(\b[18] ), .b(\a[19] ), .o(new_n241));
  nona32aa1n09x5               g146(.a(new_n219), .b(new_n236), .c(new_n241), .d(new_n228), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n218), .c(new_n143), .d(new_n215), .o1(new_n244));
  oaoi03aa1n12x5               g149(.a(new_n232), .b(new_n233), .c(new_n228), .o1(new_n245));
  oai013aa1d12x5               g150(.a(new_n245), .b(new_n224), .c(new_n223), .d(new_n236), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nor042aa1d18x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  nanp02aa1n12x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  norb02aa1n06x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n244), .c(new_n247), .out0(\s[21] ));
  inv000aa1d42x5               g156(.a(new_n248), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n250), .b(new_n246), .c(new_n207), .d(new_n243), .o1(new_n253));
  nor002aa1d32x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  nand42aa1d28x5               g159(.a(\b[21] ), .b(\a[22] ), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n254), .b(new_n255), .out0(new_n256));
  aoi012aa1n03x5               g161(.a(new_n256), .b(new_n253), .c(new_n252), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n250), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n244), .c(new_n247), .o1(new_n259));
  nano22aa1n03x5               g164(.a(new_n259), .b(new_n252), .c(new_n256), .out0(new_n260));
  nor002aa1n02x5               g165(.a(new_n257), .b(new_n260), .o1(\s[22] ));
  nano23aa1n03x7               g166(.a(new_n248), .b(new_n254), .c(new_n255), .d(new_n249), .out0(new_n262));
  norb02aa1n03x5               g167(.a(new_n262), .b(new_n242), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n218), .c(new_n143), .d(new_n215), .o1(new_n264));
  inv020aa1n04x5               g169(.a(new_n254), .o1(new_n265));
  oai112aa1n06x5               g170(.a(new_n265), .b(new_n255), .c(\b[20] ), .d(\a[21] ), .o1(new_n266));
  aoi022aa1n06x5               g171(.a(new_n246), .b(new_n262), .c(new_n255), .d(new_n266), .o1(new_n267));
  inv040aa1d32x5               g172(.a(\a[23] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(\b[22] ), .o1(new_n269));
  nand02aa1d16x5               g174(.a(new_n269), .b(new_n268), .o1(new_n270));
  nand42aa1n20x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  nand22aa1n12x5               g176(.a(new_n270), .b(new_n271), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n264), .c(new_n267), .out0(\s[23] ));
  inv040aa1n03x5               g179(.a(new_n267), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n273), .b(new_n275), .c(new_n207), .d(new_n263), .o1(new_n276));
  nor002aa1d24x5               g181(.a(\b[23] ), .b(\a[24] ), .o1(new_n277));
  nand02aa1d24x5               g182(.a(\b[23] ), .b(\a[24] ), .o1(new_n278));
  nanb02aa1n02x5               g183(.a(new_n277), .b(new_n278), .out0(new_n279));
  tech160nm_fiaoi012aa1n03p5x5 g184(.a(new_n279), .b(new_n276), .c(new_n270), .o1(new_n280));
  tech160nm_fiaoi012aa1n03p5x5 g185(.a(new_n272), .b(new_n264), .c(new_n267), .o1(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n270), .c(new_n279), .out0(new_n282));
  norp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[24] ));
  nanb03aa1n03x5               g188(.a(new_n277), .b(new_n278), .c(new_n271), .out0(new_n284));
  nanb03aa1n03x5               g189(.a(new_n284), .b(new_n262), .c(new_n270), .out0(new_n285));
  nor042aa1n02x5               g190(.a(new_n242), .b(new_n285), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n218), .c(new_n143), .d(new_n215), .o1(new_n287));
  nona22aa1n09x5               g192(.a(new_n225), .b(new_n223), .c(new_n236), .out0(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[24] ), .b(\b[23] ), .c(new_n270), .o1(new_n289));
  tech160nm_fioai012aa1n04x5   g194(.a(new_n255), .b(new_n254), .c(new_n248), .o1(new_n290));
  nor003aa1n02x5               g195(.a(new_n290), .b(new_n279), .c(new_n272), .o1(new_n291));
  nor042aa1n06x5               g196(.a(new_n291), .b(new_n289), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n285), .c(new_n288), .d(new_n245), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  xnrc02aa1n12x5               g199(.a(\b[24] ), .b(\a[25] ), .out0(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  xnbna2aa1n03x5               g201(.a(new_n296), .b(new_n287), .c(new_n294), .out0(\s[25] ));
  nor042aa1n03x5               g202(.a(\b[24] ), .b(\a[25] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n296), .b(new_n293), .c(new_n207), .d(new_n286), .o1(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[25] ), .b(\a[26] ), .out0(new_n301));
  aoi012aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n299), .o1(new_n302));
  tech160nm_fiaoi012aa1n03p5x5 g207(.a(new_n295), .b(new_n287), .c(new_n294), .o1(new_n303));
  nano22aa1n03x7               g208(.a(new_n303), .b(new_n299), .c(new_n301), .out0(new_n304));
  nor002aa1n02x5               g209(.a(new_n302), .b(new_n304), .o1(\s[26] ));
  nano23aa1n03x7               g210(.a(new_n266), .b(new_n284), .c(new_n270), .d(new_n249), .out0(new_n306));
  nor042aa1n06x5               g211(.a(new_n301), .b(new_n295), .o1(new_n307));
  nano22aa1n12x5               g212(.a(new_n242), .b(new_n306), .c(new_n307), .out0(new_n308));
  aoai13aa1n06x5               g213(.a(new_n308), .b(new_n218), .c(new_n143), .d(new_n215), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[26] ), .b(\b[25] ), .c(new_n299), .carry(new_n310));
  aobi12aa1n06x5               g215(.a(new_n310), .b(new_n293), .c(new_n307), .out0(new_n311));
  xorc02aa1n12x5               g216(.a(\a[27] ), .b(\b[26] ), .out0(new_n312));
  xnbna2aa1n03x5               g217(.a(new_n312), .b(new_n309), .c(new_n311), .out0(\s[27] ));
  nor042aa1n03x5               g218(.a(\b[26] ), .b(\a[27] ), .o1(new_n314));
  inv040aa1n03x5               g219(.a(new_n314), .o1(new_n315));
  nand22aa1n12x5               g220(.a(new_n246), .b(new_n306), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n307), .o1(new_n317));
  aoai13aa1n12x5               g222(.a(new_n310), .b(new_n317), .c(new_n316), .d(new_n292), .o1(new_n318));
  aoai13aa1n06x5               g223(.a(new_n312), .b(new_n318), .c(new_n207), .d(new_n308), .o1(new_n319));
  xnrc02aa1n12x5               g224(.a(\b[27] ), .b(\a[28] ), .out0(new_n320));
  tech160nm_fiaoi012aa1n05x5   g225(.a(new_n320), .b(new_n319), .c(new_n315), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n312), .o1(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(new_n309), .c(new_n311), .o1(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n315), .c(new_n320), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(\s[28] ));
  norb02aa1n09x5               g230(.a(new_n312), .b(new_n320), .out0(new_n326));
  aoai13aa1n06x5               g231(.a(new_n326), .b(new_n318), .c(new_n207), .d(new_n308), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[28] ), .b(\b[27] ), .c(new_n315), .carry(new_n328));
  xnrc02aa1n02x5               g233(.a(\b[28] ), .b(\a[29] ), .out0(new_n329));
  tech160nm_fiaoi012aa1n05x5   g234(.a(new_n329), .b(new_n327), .c(new_n328), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n326), .o1(new_n331));
  aoi012aa1n02x5               g236(.a(new_n331), .b(new_n309), .c(new_n311), .o1(new_n332));
  nano22aa1n02x4               g237(.a(new_n332), .b(new_n328), .c(new_n329), .out0(new_n333));
  norp02aa1n03x5               g238(.a(new_n330), .b(new_n333), .o1(\s[29] ));
  xorb03aa1n02x5               g239(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g240(.a(new_n312), .b(new_n329), .c(new_n320), .out0(new_n336));
  aoai13aa1n06x5               g241(.a(new_n336), .b(new_n318), .c(new_n207), .d(new_n308), .o1(new_n337));
  oao003aa1n02x5               g242(.a(\a[29] ), .b(\b[28] ), .c(new_n328), .carry(new_n338));
  xnrc02aa1n02x5               g243(.a(\b[29] ), .b(\a[30] ), .out0(new_n339));
  tech160nm_fiaoi012aa1n05x5   g244(.a(new_n339), .b(new_n337), .c(new_n338), .o1(new_n340));
  inv000aa1d42x5               g245(.a(new_n336), .o1(new_n341));
  tech160nm_fiaoi012aa1n02p5x5 g246(.a(new_n341), .b(new_n309), .c(new_n311), .o1(new_n342));
  nano22aa1n02x4               g247(.a(new_n342), .b(new_n338), .c(new_n339), .out0(new_n343));
  nor002aa1n02x5               g248(.a(new_n340), .b(new_n343), .o1(\s[30] ));
  xnrc02aa1n02x5               g249(.a(\b[30] ), .b(\a[31] ), .out0(new_n345));
  nona32aa1n02x4               g250(.a(new_n312), .b(new_n339), .c(new_n329), .d(new_n320), .out0(new_n346));
  inv000aa1n02x5               g251(.a(new_n346), .o1(new_n347));
  aoai13aa1n06x5               g252(.a(new_n347), .b(new_n318), .c(new_n207), .d(new_n308), .o1(new_n348));
  oao003aa1n02x5               g253(.a(\a[30] ), .b(\b[29] ), .c(new_n338), .carry(new_n349));
  tech160nm_fiaoi012aa1n05x5   g254(.a(new_n345), .b(new_n348), .c(new_n349), .o1(new_n350));
  tech160nm_fiaoi012aa1n02p5x5 g255(.a(new_n346), .b(new_n309), .c(new_n311), .o1(new_n351));
  nano22aa1n02x4               g256(.a(new_n351), .b(new_n345), .c(new_n349), .out0(new_n352));
  norp02aa1n03x5               g257(.a(new_n350), .b(new_n352), .o1(\s[31] ));
  xnbna2aa1n03x5               g258(.a(new_n134), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  norb02aa1n02x5               g259(.a(new_n105), .b(new_n106), .out0(new_n355));
  aoi012aa1n02x5               g260(.a(new_n99), .b(new_n135), .c(new_n134), .o1(new_n356));
  oai012aa1n02x5               g261(.a(new_n108), .b(new_n356), .c(new_n355), .o1(\s[4] ));
  norb02aa1n02x5               g262(.a(new_n113), .b(new_n124), .out0(new_n358));
  xobna2aa1n03x5               g263(.a(new_n358), .b(new_n108), .c(new_n105), .out0(\s[5] ));
  norb02aa1n02x5               g264(.a(new_n109), .b(new_n123), .out0(new_n360));
  nanp03aa1n02x5               g265(.a(new_n108), .b(new_n105), .c(new_n358), .o1(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n360), .b(new_n361), .c(new_n115), .out0(\s[6] ));
  nanp03aa1n02x5               g267(.a(new_n361), .b(new_n115), .c(new_n360), .o1(new_n363));
  nanp02aa1n02x5               g268(.a(new_n363), .b(new_n109), .o1(new_n364));
  xnrb03aa1n02x5               g269(.a(new_n364), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g270(.a(\a[7] ), .b(\b[6] ), .c(new_n364), .o1(new_n366));
  xorb03aa1n02x5               g271(.a(new_n366), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g272(.a(new_n161), .b(new_n127), .c(new_n98), .out0(\s[9] ));
endmodule


