// Benchmark "adder" written by ABC on Thu Jul 18 11:59:23 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n314, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n332, new_n333, new_n334, new_n336, new_n337, new_n339,
    new_n340, new_n341, new_n342, new_n344, new_n345, new_n346, new_n348,
    new_n350, new_n351, new_n352, new_n353, new_n355, new_n356;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nanp02aa1n06x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  oai112aa1n06x5               g003(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n99));
  nand42aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nano32aa1n03x7               g006(.a(new_n101), .b(new_n99), .c(new_n100), .d(new_n98), .out0(new_n102));
  oai022aa1d24x5               g007(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  inv040aa1d32x5               g010(.a(\a[5] ), .o1(new_n106));
  inv040aa1d32x5               g011(.a(\b[4] ), .o1(new_n107));
  nand42aa1n02x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  nand23aa1n03x5               g013(.a(new_n108), .b(new_n104), .c(new_n105), .o1(new_n109));
  aoi022aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n110));
  nor022aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor042aa1n02x5               g017(.a(new_n112), .b(new_n111), .o1(new_n113));
  xnrc02aa1n12x5               g018(.a(\b[6] ), .b(\a[7] ), .out0(new_n114));
  nano23aa1d12x5               g019(.a(new_n109), .b(new_n114), .c(new_n113), .d(new_n110), .out0(new_n115));
  oai012aa1n18x5               g020(.a(new_n115), .b(new_n102), .c(new_n103), .o1(new_n116));
  inv000aa1n02x5               g021(.a(new_n111), .o1(new_n117));
  nor022aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  inv020aa1n04x5               g023(.a(new_n118), .o1(new_n119));
  nand42aa1n16x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  aoai13aa1n12x5               g025(.a(new_n120), .b(new_n112), .c(new_n107), .d(new_n106), .o1(new_n121));
  aoi022aa1n12x5               g026(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n122));
  inv040aa1n02x5               g027(.a(new_n122), .o1(new_n123));
  aoai13aa1n09x5               g028(.a(new_n117), .b(new_n123), .c(new_n121), .d(new_n119), .o1(new_n124));
  inv040aa1n03x5               g029(.a(new_n124), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aob012aa1n03x5               g031(.a(new_n126), .b(new_n116), .c(new_n125), .out0(new_n127));
  xorc02aa1n12x5               g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n127), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n126), .o1(new_n130));
  aoai13aa1n06x5               g035(.a(new_n97), .b(new_n130), .c(new_n116), .d(new_n125), .o1(new_n131));
  oaoi03aa1n12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand22aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n06x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  aoai13aa1n06x5               g040(.a(new_n135), .b(new_n132), .c(new_n131), .d(new_n128), .o1(new_n136));
  aoi112aa1n02x5               g041(.a(new_n135), .b(new_n132), .c(new_n131), .d(new_n128), .o1(new_n137));
  norb02aa1n02x7               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  inv040aa1n08x5               g043(.a(new_n133), .o1(new_n139));
  nor002aa1n20x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand22aa1n09x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n03x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  nona23aa1n03x5               g047(.a(new_n136), .b(new_n141), .c(new_n140), .d(new_n133), .out0(new_n143));
  aoai13aa1n03x5               g048(.a(new_n143), .b(new_n142), .c(new_n139), .d(new_n136), .o1(\s[12] ));
  nona23aa1n09x5               g049(.a(new_n141), .b(new_n134), .c(new_n133), .d(new_n140), .out0(new_n145));
  nano22aa1d15x5               g050(.a(new_n145), .b(new_n126), .c(new_n128), .out0(new_n146));
  aob012aa1n02x5               g051(.a(new_n146), .b(new_n116), .c(new_n125), .out0(new_n147));
  inv000aa1n02x5               g052(.a(new_n146), .o1(new_n148));
  nano23aa1n03x7               g053(.a(new_n133), .b(new_n140), .c(new_n141), .d(new_n134), .out0(new_n149));
  oaoi03aa1n06x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n139), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n132), .o1(new_n151));
  aoai13aa1n12x5               g056(.a(new_n151), .b(new_n148), .c(new_n116), .d(new_n125), .o1(new_n152));
  nor002aa1d32x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand02aa1d24x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n12x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  aoi112aa1n02x5               g060(.a(new_n150), .b(new_n155), .c(new_n149), .d(new_n132), .o1(new_n156));
  aoi022aa1n02x5               g061(.a(new_n152), .b(new_n155), .c(new_n147), .d(new_n156), .o1(\s[13] ));
  tech160nm_fixnrc02aa1n03p5x5 g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n153), .c(new_n152), .d(new_n155), .o1(new_n159));
  inv000aa1d42x5               g064(.a(\b[13] ), .o1(new_n160));
  inv000aa1d42x5               g065(.a(\a[14] ), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n153), .b(new_n161), .c(new_n160), .o1(new_n162));
  oaib12aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(\a[14] ), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n159), .b(new_n163), .c(new_n155), .d(new_n152), .o1(\s[14] ));
  inv040aa1n08x5               g069(.a(new_n153), .o1(new_n165));
  nano22aa1n06x5               g070(.a(new_n158), .b(new_n165), .c(new_n154), .out0(new_n166));
  oaoi03aa1n12x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n165), .o1(new_n167));
  xorc02aa1n12x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n167), .c(new_n152), .d(new_n166), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n168), .b(new_n167), .c(new_n152), .d(new_n166), .o1(new_n170));
  norb02aa1n03x4               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  nor042aa1n03x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  xorc02aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .out0(new_n174));
  inv000aa1d42x5               g079(.a(\a[16] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\b[15] ), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n172), .b(new_n175), .c(new_n176), .o1(new_n177));
  oai112aa1n03x5               g082(.a(new_n169), .b(new_n177), .c(new_n176), .d(new_n175), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n174), .c(new_n173), .d(new_n169), .o1(\s[16] ));
  xorc02aa1n03x5               g084(.a(\a[14] ), .b(\b[13] ), .out0(new_n180));
  nand22aa1n03x5               g085(.a(new_n180), .b(new_n155), .o1(new_n181));
  nand22aa1n02x5               g086(.a(new_n174), .b(new_n168), .o1(new_n182));
  nona22aa1d24x5               g087(.a(new_n146), .b(new_n181), .c(new_n182), .out0(new_n183));
  aoi012aa1d18x5               g088(.a(new_n183), .b(new_n116), .c(new_n125), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n167), .o1(new_n185));
  aoai13aa1n06x5               g090(.a(new_n166), .b(new_n150), .c(new_n149), .d(new_n132), .o1(new_n186));
  tech160nm_fioaoi03aa1n03p5x5 g091(.a(new_n175), .b(new_n176), .c(new_n172), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n182), .c(new_n186), .d(new_n185), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  oaih12aa1n02x5               g094(.a(new_n189), .b(new_n188), .c(new_n184), .o1(new_n190));
  norp02aa1n02x5               g095(.a(\b[9] ), .b(\a[10] ), .o1(new_n191));
  aoi112aa1n03x5               g096(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n192));
  oai112aa1n03x5               g097(.a(new_n135), .b(new_n142), .c(new_n192), .d(new_n191), .o1(new_n193));
  inv000aa1n02x5               g098(.a(new_n150), .o1(new_n194));
  aoai13aa1n02x5               g099(.a(new_n185), .b(new_n181), .c(new_n193), .d(new_n194), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n182), .b(new_n195), .out0(new_n196));
  nano23aa1n02x4               g101(.a(new_n189), .b(new_n184), .c(new_n196), .d(new_n187), .out0(new_n197));
  norb02aa1n02x7               g102(.a(new_n190), .b(new_n197), .out0(\s[17] ));
  xorc02aa1n12x5               g103(.a(\a[18] ), .b(\b[17] ), .out0(new_n199));
  nor002aa1n06x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  oaoi13aa1n02x5               g105(.a(new_n200), .b(new_n189), .c(new_n188), .d(new_n184), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\a[18] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\b[17] ), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n200), .b(new_n202), .c(new_n203), .o1(new_n204));
  oai112aa1n03x5               g109(.a(new_n190), .b(new_n204), .c(new_n203), .d(new_n202), .o1(new_n205));
  oaih12aa1n02x5               g110(.a(new_n205), .b(new_n201), .c(new_n199), .o1(\s[18] ));
  and002aa1n02x5               g111(.a(new_n199), .b(new_n189), .o(new_n207));
  oaih12aa1n02x5               g112(.a(new_n207), .b(new_n188), .c(new_n184), .o1(new_n208));
  oaoi03aa1n03x5               g113(.a(new_n202), .b(new_n203), .c(new_n200), .o1(new_n209));
  nor002aa1d32x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n06x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n208), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n08x5               g119(.a(new_n210), .o1(new_n215));
  oaoi13aa1n12x5               g120(.a(new_n124), .b(new_n115), .c(new_n102), .d(new_n103), .o1(new_n216));
  oai112aa1n06x5               g121(.a(new_n196), .b(new_n187), .c(new_n216), .d(new_n183), .o1(new_n217));
  oao003aa1n02x5               g122(.a(new_n202), .b(new_n203), .c(new_n200), .carry(new_n218));
  aoai13aa1n03x5               g123(.a(new_n212), .b(new_n218), .c(new_n217), .d(new_n207), .o1(new_n219));
  nor022aa1n16x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand42aa1n04x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  norb03aa1n02x5               g127(.a(new_n221), .b(new_n210), .c(new_n220), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n219), .b(new_n223), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n222), .c(new_n219), .d(new_n215), .o1(\s[20] ));
  nona23aa1n09x5               g130(.a(new_n221), .b(new_n211), .c(new_n210), .d(new_n220), .out0(new_n226));
  nano22aa1n03x7               g131(.a(new_n226), .b(new_n189), .c(new_n199), .out0(new_n227));
  oai012aa1n02x5               g132(.a(new_n227), .b(new_n188), .c(new_n184), .o1(new_n228));
  oaoi03aa1n09x5               g133(.a(\a[20] ), .b(\b[19] ), .c(new_n215), .o1(new_n229));
  inv000aa1n02x5               g134(.a(new_n229), .o1(new_n230));
  oai012aa1n06x5               g135(.a(new_n230), .b(new_n226), .c(new_n209), .o1(new_n231));
  nor042aa1n06x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n231), .c(new_n217), .d(new_n227), .o1(new_n235));
  nano23aa1n03x7               g140(.a(new_n210), .b(new_n220), .c(new_n221), .d(new_n211), .out0(new_n236));
  aoi112aa1n02x5               g141(.a(new_n229), .b(new_n234), .c(new_n236), .d(new_n218), .o1(new_n237));
  aobi12aa1n03x7               g142(.a(new_n235), .b(new_n237), .c(new_n228), .out0(\s[21] ));
  inv000aa1n04x5               g143(.a(new_n232), .o1(new_n239));
  nor042aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  and002aa1n12x5               g145(.a(\b[21] ), .b(\a[22] ), .o(new_n241));
  norp02aa1n02x5               g146(.a(new_n241), .b(new_n240), .o1(new_n242));
  norp03aa1n02x5               g147(.a(new_n241), .b(new_n240), .c(new_n232), .o1(new_n243));
  nanp02aa1n03x5               g148(.a(new_n235), .b(new_n243), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n242), .c(new_n235), .d(new_n239), .o1(\s[22] ));
  nano23aa1n06x5               g150(.a(new_n241), .b(new_n240), .c(new_n239), .d(new_n233), .out0(new_n246));
  nano32aa1n02x4               g151(.a(new_n226), .b(new_n246), .c(new_n189), .d(new_n199), .out0(new_n247));
  oai012aa1n02x5               g152(.a(new_n247), .b(new_n188), .c(new_n184), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n246), .b(new_n229), .c(new_n236), .d(new_n218), .o1(new_n249));
  oab012aa1n09x5               g154(.a(new_n240), .b(new_n239), .c(new_n241), .out0(new_n250));
  nanp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n217), .d(new_n247), .o1(new_n253));
  inv020aa1n02x5               g158(.a(new_n250), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n231), .d(new_n246), .o1(new_n255));
  aobi12aa1n03x7               g160(.a(new_n253), .b(new_n255), .c(new_n248), .out0(\s[23] ));
  nor042aa1n04x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nor042aa1n04x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  and002aa1n12x5               g164(.a(\b[23] ), .b(\a[24] ), .o(new_n260));
  nor042aa1n02x5               g165(.a(new_n260), .b(new_n259), .o1(new_n261));
  norp03aa1n02x5               g166(.a(new_n260), .b(new_n259), .c(new_n257), .o1(new_n262));
  nanp02aa1n06x5               g167(.a(new_n253), .b(new_n262), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n261), .c(new_n253), .d(new_n258), .o1(\s[24] ));
  inv000aa1n02x5               g169(.a(new_n227), .o1(new_n265));
  and002aa1n03x5               g170(.a(new_n252), .b(new_n261), .o(new_n266));
  nano22aa1n02x4               g171(.a(new_n265), .b(new_n246), .c(new_n266), .out0(new_n267));
  oai012aa1n02x5               g172(.a(new_n267), .b(new_n188), .c(new_n184), .o1(new_n268));
  inv030aa1n02x5               g173(.a(new_n266), .o1(new_n269));
  oab012aa1n04x5               g174(.a(new_n259), .b(new_n258), .c(new_n260), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n269), .c(new_n249), .d(new_n250), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n217), .d(new_n267), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n266), .b(new_n254), .c(new_n231), .d(new_n246), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n272), .o1(new_n275));
  and003aa1n02x5               g180(.a(new_n274), .b(new_n275), .c(new_n270), .o(new_n276));
  aobi12aa1n03x7               g181(.a(new_n273), .b(new_n276), .c(new_n268), .out0(\s[25] ));
  nor042aa1n03x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  norp02aa1n02x5               g184(.a(\b[25] ), .b(\a[26] ), .o1(new_n280));
  and002aa1n02x7               g185(.a(\b[25] ), .b(\a[26] ), .o(new_n281));
  nor002aa1n02x5               g186(.a(new_n281), .b(new_n280), .o1(new_n282));
  norp03aa1n02x5               g187(.a(new_n281), .b(new_n280), .c(new_n278), .o1(new_n283));
  nanp02aa1n06x5               g188(.a(new_n273), .b(new_n283), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n273), .d(new_n279), .o1(\s[26] ));
  and002aa1n06x5               g190(.a(new_n272), .b(new_n282), .o(new_n286));
  nano32aa1n03x7               g191(.a(new_n265), .b(new_n286), .c(new_n246), .d(new_n266), .out0(new_n287));
  oai012aa1n12x5               g192(.a(new_n287), .b(new_n188), .c(new_n184), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n286), .o1(new_n289));
  oab012aa1n02x4               g194(.a(new_n280), .b(new_n279), .c(new_n281), .out0(new_n290));
  aoai13aa1n02x7               g195(.a(new_n290), .b(new_n289), .c(new_n274), .d(new_n270), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n217), .d(new_n287), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n290), .o1(new_n294));
  aoi112aa1n02x5               g199(.a(new_n292), .b(new_n294), .c(new_n271), .d(new_n286), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n293), .b(new_n295), .c(new_n288), .out0(\s[27] ));
  norp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  tech160nm_fixorc02aa1n02p5x5 g203(.a(\a[28] ), .b(\b[27] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n294), .b(new_n271), .c(new_n286), .o1(new_n300));
  inv040aa1n02x5               g205(.a(new_n292), .o1(new_n301));
  oai022aa1n02x5               g206(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(\a[28] ), .c(\b[27] ), .o1(new_n303));
  aoai13aa1n04x5               g208(.a(new_n303), .b(new_n301), .c(new_n300), .d(new_n288), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n293), .d(new_n298), .o1(\s[28] ));
  and002aa1n02x5               g210(.a(new_n299), .b(new_n292), .o(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n291), .c(new_n217), .d(new_n287), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  aob012aa1n02x5               g213(.a(new_n302), .b(\b[27] ), .c(\a[28] ), .out0(new_n309));
  aoai13aa1n02x7               g214(.a(new_n309), .b(new_n308), .c(new_n300), .d(new_n288), .o1(new_n310));
  tech160nm_fixorc02aa1n03p5x5 g215(.a(\a[29] ), .b(\b[28] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n309), .b(new_n311), .out0(new_n312));
  aoi022aa1n02x7               g217(.a(new_n310), .b(new_n311), .c(new_n307), .d(new_n312), .o1(\s[29] ));
  nanp02aa1n02x5               g218(.a(\b[0] ), .b(\a[1] ), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g220(.a(new_n301), .b(new_n299), .c(new_n311), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n291), .c(new_n217), .d(new_n287), .o1(new_n317));
  inv000aa1n02x5               g222(.a(new_n316), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n318), .c(new_n300), .d(new_n288), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n319), .b(new_n321), .out0(new_n322));
  aoi022aa1n02x7               g227(.a(new_n320), .b(new_n321), .c(new_n317), .d(new_n322), .o1(\s[30] ));
  nano32aa1n06x5               g228(.a(new_n301), .b(new_n321), .c(new_n299), .d(new_n311), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n291), .c(new_n217), .d(new_n287), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n324), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n319), .carry(new_n327));
  aoai13aa1n02x7               g232(.a(new_n327), .b(new_n326), .c(new_n300), .d(new_n288), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[31] ), .b(\b[30] ), .out0(new_n329));
  norb02aa1n02x5               g234(.a(new_n327), .b(new_n329), .out0(new_n330));
  aoi022aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n325), .d(new_n330), .o1(\s[31] ));
  tech160nm_fiaoi012aa1n03p5x5 g236(.a(new_n99), .b(\a[2] ), .c(\b[1] ), .o1(new_n332));
  nanb03aa1n03x5               g237(.a(new_n101), .b(new_n98), .c(new_n100), .out0(new_n333));
  aboi22aa1n03x5               g238(.a(new_n101), .b(new_n100), .c(new_n99), .d(new_n98), .out0(new_n334));
  oab012aa1n02x4               g239(.a(new_n334), .b(new_n332), .c(new_n333), .out0(\s[3] ));
  xorc02aa1n02x5               g240(.a(\a[4] ), .b(\b[3] ), .out0(new_n336));
  aoi113aa1n02x5               g241(.a(new_n336), .b(new_n101), .c(new_n99), .d(new_n100), .e(new_n98), .o1(new_n337));
  oaoi13aa1n02x5               g242(.a(new_n337), .b(new_n336), .c(new_n102), .d(new_n103), .o1(\s[4] ));
  oab012aa1n03x5               g243(.a(new_n103), .b(new_n332), .c(new_n333), .out0(new_n339));
  nanp02aa1n02x5               g244(.a(new_n108), .b(new_n105), .o1(new_n340));
  and002aa1n02x5               g245(.a(\b[3] ), .b(\a[4] ), .o(new_n341));
  nano23aa1n06x5               g246(.a(new_n339), .b(new_n341), .c(new_n105), .d(new_n108), .out0(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n342), .b(new_n340), .c(new_n339), .d(new_n341), .o1(\s[5] ));
  norb02aa1n02x5               g248(.a(new_n120), .b(new_n112), .out0(new_n344));
  aoai13aa1n06x5               g249(.a(new_n344), .b(new_n342), .c(new_n106), .d(new_n107), .o1(new_n345));
  aoi112aa1n02x5               g250(.a(new_n342), .b(new_n344), .c(new_n106), .d(new_n107), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n346), .out0(\s[6] ));
  inv000aa1d42x5               g252(.a(new_n114), .o1(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n348), .b(new_n345), .c(new_n121), .out0(\s[7] ));
  aob012aa1n02x5               g254(.a(new_n348), .b(new_n345), .c(new_n121), .out0(new_n350));
  aoai13aa1n02x5               g255(.a(new_n119), .b(new_n114), .c(new_n345), .d(new_n121), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n104), .b(new_n111), .out0(new_n352));
  aoib12aa1n02x5               g257(.a(new_n118), .b(new_n104), .c(new_n111), .out0(new_n353));
  aoi022aa1n03x5               g258(.a(new_n351), .b(new_n352), .c(new_n350), .d(new_n353), .o1(\s[8] ));
  nanp02aa1n02x5               g259(.a(new_n121), .b(new_n119), .o1(new_n355));
  aoi112aa1n02x5               g260(.a(new_n126), .b(new_n111), .c(new_n355), .d(new_n122), .o1(new_n356));
  aboi22aa1n03x5               g261(.a(new_n216), .b(new_n126), .c(new_n356), .d(new_n116), .out0(\s[9] ));
endmodule


