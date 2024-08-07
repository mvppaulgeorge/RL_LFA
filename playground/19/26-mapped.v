// Benchmark "adder" written by ABC on Wed Jul 17 21:55:11 2024

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
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n05x5   g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  nor042aa1n12x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\a[4] ), .o1(new_n100));
  inv030aa1d32x5               g005(.a(\b[3] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nand42aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1n04x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  xnrc02aa1n12x5               g009(.a(\b[2] ), .b(\a[3] ), .out0(new_n105));
  nanp02aa1n04x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand22aa1n12x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nor042aa1n06x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  oaih12aa1n12x5               g013(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n109));
  aoi112aa1n03x5               g014(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n110));
  norb02aa1n06x4               g015(.a(new_n102), .b(new_n110), .out0(new_n111));
  oai013aa1d12x5               g016(.a(new_n111), .b(new_n105), .c(new_n109), .d(new_n104), .o1(new_n112));
  xnrc02aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  nor022aa1n16x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand02aa1d06x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor022aa1n16x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand02aa1n04x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1d18x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  tech160nm_fixnrc02aa1n02p5x5 g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  nor043aa1n06x5               g024(.a(new_n118), .b(new_n119), .c(new_n113), .o1(new_n120));
  oaih12aa1n02x5               g025(.a(new_n115), .b(new_n116), .c(new_n114), .o1(new_n121));
  inv040aa1d32x5               g026(.a(\a[5] ), .o1(new_n122));
  inv040aa1d28x5               g027(.a(\b[4] ), .o1(new_n123));
  nanp02aa1n12x5               g028(.a(new_n123), .b(new_n122), .o1(new_n124));
  oaoi03aa1n09x5               g029(.a(\a[6] ), .b(\b[5] ), .c(new_n124), .o1(new_n125));
  oaib12aa1n09x5               g030(.a(new_n121), .b(new_n118), .c(new_n125), .out0(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(\a[10] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(\b[9] ), .o1(new_n131));
  nanp03aa1n02x5               g036(.a(new_n128), .b(new_n97), .c(new_n99), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1n10x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  oai112aa1n03x5               g040(.a(new_n132), .b(new_n135), .c(new_n131), .d(new_n130), .o1(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n135), .b(new_n132), .c(new_n130), .d(new_n131), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  inv040aa1n06x5               g043(.a(new_n133), .o1(new_n139));
  nor002aa1d32x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n08x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[12] ));
  nand22aa1n03x5               g048(.a(new_n112), .b(new_n120), .o1(new_n144));
  oai022aa1n02x5               g049(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n145));
  aboi22aa1n03x5               g050(.a(new_n118), .b(new_n125), .c(new_n115), .d(new_n145), .out0(new_n146));
  nano23aa1n09x5               g051(.a(new_n133), .b(new_n140), .c(new_n141), .d(new_n134), .out0(new_n147));
  nand23aa1n03x5               g052(.a(new_n147), .b(new_n97), .c(new_n127), .o1(new_n148));
  nona23aa1n09x5               g053(.a(new_n141), .b(new_n134), .c(new_n133), .d(new_n140), .out0(new_n149));
  oaoi03aa1n09x5               g054(.a(new_n130), .b(new_n131), .c(new_n98), .o1(new_n150));
  oaoi03aa1n09x5               g055(.a(\a[12] ), .b(\b[11] ), .c(new_n139), .o1(new_n151));
  oabi12aa1n18x5               g056(.a(new_n151), .b(new_n149), .c(new_n150), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n148), .c(new_n144), .d(new_n146), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n24x5               g060(.a(\a[13] ), .b(\b[12] ), .o(new_n156));
  tech160nm_fixnrc02aa1n05x5   g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  nanb02aa1n02x5               g062(.a(new_n157), .b(new_n154), .out0(new_n158));
  xnrc02aa1n12x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  xobna2aa1n03x5               g064(.a(new_n159), .b(new_n158), .c(new_n156), .out0(\s[14] ));
  xorc02aa1n12x5               g065(.a(\a[15] ), .b(\b[14] ), .out0(new_n161));
  nor042aa1n06x5               g066(.a(new_n159), .b(new_n157), .o1(new_n162));
  oaoi03aa1n09x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n161), .b(new_n163), .c(new_n154), .d(new_n162), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n161), .b(new_n163), .c(new_n154), .d(new_n162), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  orn002aa1n02x5               g071(.a(\a[15] ), .b(\b[14] ), .o(new_n167));
  xorc02aa1n12x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n167), .out0(\s[16] ));
  tech160nm_fiaoi012aa1n05x5   g074(.a(new_n126), .b(new_n112), .c(new_n120), .o1(new_n170));
  and002aa1n12x5               g075(.a(new_n168), .b(new_n161), .o(new_n171));
  nanb03aa1n12x5               g076(.a(new_n148), .b(new_n171), .c(new_n162), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n171), .b(new_n163), .c(new_n152), .d(new_n162), .o1(new_n173));
  oao003aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .c(new_n167), .carry(new_n174));
  oai112aa1n06x5               g079(.a(new_n173), .b(new_n174), .c(new_n172), .d(new_n170), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g081(.a(\a[18] ), .o1(new_n177));
  inv040aa1d32x5               g082(.a(\a[17] ), .o1(new_n178));
  inv030aa1d32x5               g083(.a(\b[16] ), .o1(new_n179));
  oaoi03aa1n03x5               g084(.a(new_n178), .b(new_n179), .c(new_n175), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(new_n177), .out0(\s[18] ));
  inv030aa1n06x5               g086(.a(new_n170), .o1(new_n182));
  inv040aa1n04x5               g087(.a(new_n172), .o1(new_n183));
  inv000aa1n02x5               g088(.a(new_n163), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n171), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n186));
  aoai13aa1n04x5               g091(.a(new_n162), .b(new_n151), .c(new_n147), .d(new_n186), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n174), .b(new_n185), .c(new_n187), .d(new_n184), .o1(new_n188));
  xroi22aa1d06x4               g093(.a(new_n178), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n188), .c(new_n183), .d(new_n182), .o1(new_n190));
  oai022aa1n04x7               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  oaib12aa1n09x5               g096(.a(new_n191), .b(new_n177), .c(\b[17] ), .out0(new_n192));
  nor002aa1d32x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand42aa1d28x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n190), .c(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g103(.a(new_n193), .o1(new_n199));
  tech160nm_fiaoi012aa1n02p5x5 g104(.a(new_n195), .b(new_n190), .c(new_n192), .o1(new_n200));
  nor002aa1n20x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand42aa1d28x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  nano22aa1n02x4               g108(.a(new_n200), .b(new_n199), .c(new_n203), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(new_n179), .b(new_n178), .o1(new_n205));
  oaoi03aa1n06x5               g110(.a(\a[18] ), .b(\b[17] ), .c(new_n205), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n196), .b(new_n206), .c(new_n175), .d(new_n189), .o1(new_n207));
  aoi012aa1n03x5               g112(.a(new_n203), .b(new_n207), .c(new_n199), .o1(new_n208));
  nor002aa1n02x5               g113(.a(new_n208), .b(new_n204), .o1(\s[20] ));
  nano23aa1n09x5               g114(.a(new_n193), .b(new_n201), .c(new_n202), .d(new_n194), .out0(new_n210));
  nand02aa1d04x5               g115(.a(new_n189), .b(new_n210), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n188), .c(new_n183), .d(new_n182), .o1(new_n213));
  nona23aa1n09x5               g118(.a(new_n202), .b(new_n194), .c(new_n193), .d(new_n201), .out0(new_n214));
  aoi012aa1n12x5               g119(.a(new_n201), .b(new_n193), .c(new_n202), .o1(new_n215));
  oai012aa1n18x5               g120(.a(new_n215), .b(new_n214), .c(new_n192), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nor002aa1d32x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nand42aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n213), .c(new_n217), .out0(\s[21] ));
  inv000aa1d42x5               g126(.a(new_n218), .o1(new_n222));
  aobi12aa1n02x5               g127(.a(new_n220), .b(new_n213), .c(new_n217), .out0(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  nano22aa1n02x4               g129(.a(new_n223), .b(new_n222), .c(new_n224), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n220), .b(new_n216), .c(new_n175), .d(new_n212), .o1(new_n226));
  aoi012aa1n02x7               g131(.a(new_n224), .b(new_n226), .c(new_n222), .o1(new_n227));
  nor002aa1n02x5               g132(.a(new_n227), .b(new_n225), .o1(\s[22] ));
  nano22aa1n06x5               g133(.a(new_n224), .b(new_n222), .c(new_n219), .out0(new_n229));
  and003aa1n02x5               g134(.a(new_n189), .b(new_n229), .c(new_n210), .o(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n188), .c(new_n183), .d(new_n182), .o1(new_n231));
  oao003aa1n12x5               g136(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .carry(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n216), .c(new_n229), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[22] ), .b(\a[23] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n234), .out0(\s[23] ));
  nor042aa1n06x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  tech160nm_fiaoi012aa1n03p5x5 g144(.a(new_n235), .b(new_n231), .c(new_n234), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  nano22aa1n03x7               g146(.a(new_n240), .b(new_n239), .c(new_n241), .out0(new_n242));
  inv030aa1n02x5               g147(.a(new_n234), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n236), .b(new_n243), .c(new_n175), .d(new_n230), .o1(new_n244));
  aoi012aa1n03x5               g149(.a(new_n241), .b(new_n244), .c(new_n239), .o1(new_n245));
  nor002aa1n02x5               g150(.a(new_n245), .b(new_n242), .o1(\s[24] ));
  nor002aa1n03x5               g151(.a(new_n241), .b(new_n235), .o1(new_n247));
  nano22aa1n03x7               g152(.a(new_n211), .b(new_n229), .c(new_n247), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n188), .c(new_n183), .d(new_n182), .o1(new_n249));
  inv020aa1n04x5               g154(.a(new_n215), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n229), .b(new_n250), .c(new_n210), .d(new_n206), .o1(new_n251));
  inv030aa1n04x5               g156(.a(new_n247), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .carry(new_n253));
  aoai13aa1n12x5               g158(.a(new_n253), .b(new_n252), .c(new_n251), .d(new_n232), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  xnrc02aa1n12x5               g160(.a(\b[24] ), .b(\a[25] ), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  xnbna2aa1n03x5               g162(.a(new_n257), .b(new_n249), .c(new_n255), .out0(\s[25] ));
  nor042aa1n03x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  tech160nm_fiaoi012aa1n04x5   g165(.a(new_n256), .b(new_n249), .c(new_n255), .o1(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[25] ), .b(\a[26] ), .out0(new_n262));
  nano22aa1n02x4               g167(.a(new_n261), .b(new_n260), .c(new_n262), .out0(new_n263));
  aoai13aa1n03x5               g168(.a(new_n257), .b(new_n254), .c(new_n175), .d(new_n248), .o1(new_n264));
  aoi012aa1n03x5               g169(.a(new_n262), .b(new_n264), .c(new_n260), .o1(new_n265));
  nor002aa1n02x5               g170(.a(new_n265), .b(new_n263), .o1(\s[26] ));
  nor042aa1n09x5               g171(.a(new_n262), .b(new_n256), .o1(new_n267));
  nano32aa1n03x7               g172(.a(new_n211), .b(new_n267), .c(new_n229), .d(new_n247), .out0(new_n268));
  aoai13aa1n09x5               g173(.a(new_n268), .b(new_n188), .c(new_n183), .d(new_n182), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n270));
  aobi12aa1n12x5               g175(.a(new_n270), .b(new_n254), .c(new_n267), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n269), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  aobi12aa1n03x5               g180(.a(new_n272), .b(new_n269), .c(new_n271), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  nano22aa1n03x5               g182(.a(new_n276), .b(new_n275), .c(new_n277), .out0(new_n278));
  aoai13aa1n03x5               g183(.a(new_n247), .b(new_n233), .c(new_n216), .d(new_n229), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n267), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n270), .b(new_n280), .c(new_n279), .d(new_n253), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n272), .b(new_n281), .c(new_n175), .d(new_n268), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n277), .b(new_n282), .c(new_n275), .o1(new_n283));
  norp02aa1n03x5               g188(.a(new_n283), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n272), .b(new_n277), .out0(new_n285));
  aobi12aa1n03x5               g190(.a(new_n285), .b(new_n269), .c(new_n271), .out0(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n286), .b(new_n287), .c(new_n288), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n285), .b(new_n281), .c(new_n175), .d(new_n268), .o1(new_n290));
  aoi012aa1n03x5               g195(.a(new_n288), .b(new_n290), .c(new_n287), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n272), .b(new_n288), .c(new_n277), .out0(new_n294));
  aobi12aa1n03x5               g199(.a(new_n294), .b(new_n269), .c(new_n271), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  aoai13aa1n02x7               g203(.a(new_n294), .b(new_n281), .c(new_n175), .d(new_n268), .o1(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n294), .b(new_n297), .out0(new_n302));
  aobi12aa1n03x5               g207(.a(new_n302), .b(new_n269), .c(new_n271), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n302), .b(new_n281), .c(new_n175), .d(new_n268), .o1(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n03x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g218(.a(new_n122), .b(new_n123), .c(new_n112), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g220(.a(\a[6] ), .b(\b[5] ), .c(new_n314), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g222(.a(new_n116), .b(new_n316), .c(new_n117), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g224(.a(new_n127), .b(new_n144), .c(new_n146), .out0(\s[9] ));
endmodule


