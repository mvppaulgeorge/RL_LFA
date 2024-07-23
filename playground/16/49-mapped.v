// Benchmark "adder" written by ABC on Wed Jul 17 20:37:07 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n322, new_n324, new_n326,
    new_n327, new_n329, new_n331, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  oab012aa1n04x5               g004(.a(new_n99), .b(\a[4] ), .c(\b[3] ), .out0(new_n100));
  nand02aa1n06x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanb02aa1n12x5               g006(.a(new_n99), .b(new_n101), .out0(new_n102));
  nor002aa1n04x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  inv030aa1n03x5               g008(.a(new_n103), .o1(new_n104));
  nand02aa1n06x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  aob012aa1n12x5               g010(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(new_n106));
  aoai13aa1n12x5               g011(.a(new_n100), .b(new_n102), .c(new_n106), .d(new_n104), .o1(new_n107));
  nor042aa1d18x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand02aa1d16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n109), .b(\b[6] ), .c(\a[7] ), .o1(new_n110));
  aoi112aa1n03x5               g015(.a(new_n110), .b(new_n108), .c(\a[8] ), .d(\b[7] ), .o1(new_n111));
  inv040aa1d32x5               g016(.a(\a[5] ), .o1(new_n112));
  inv040aa1n16x5               g017(.a(\b[4] ), .o1(new_n113));
  nand22aa1n12x5               g018(.a(new_n113), .b(new_n112), .o1(new_n114));
  nand42aa1n06x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nand42aa1n08x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nand02aa1d28x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  oai012aa1d24x5               g022(.a(new_n117), .b(\b[7] ), .c(\a[8] ), .o1(new_n118));
  nano32aa1n03x7               g023(.a(new_n118), .b(new_n114), .c(new_n115), .d(new_n116), .out0(new_n119));
  nanp03aa1d12x5               g024(.a(new_n107), .b(new_n119), .c(new_n111), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  oai022aa1n02x5               g026(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n122));
  inv030aa1n06x5               g027(.a(new_n108), .o1(new_n123));
  aoi112aa1n06x5               g028(.a(new_n110), .b(new_n118), .c(new_n123), .d(new_n114), .o1(new_n124));
  oai012aa1n12x5               g029(.a(new_n121), .b(new_n124), .c(new_n122), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n98), .b(new_n126), .c(new_n120), .d(new_n125), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xorc02aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  oaoi03aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1d28x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n130), .c(new_n127), .d(new_n129), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n133), .b(new_n130), .c(new_n127), .d(new_n129), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  inv000aa1n02x5               g041(.a(new_n131), .o1(new_n137));
  nor002aa1d32x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n08x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  xobna2aa1n03x5               g045(.a(new_n140), .b(new_n134), .c(new_n137), .out0(\s[12] ));
  inv000aa1d42x5               g046(.a(\a[10] ), .o1(new_n142));
  inv000aa1d42x5               g047(.a(\b[9] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(new_n143), .b(new_n142), .o1(new_n144));
  nand02aa1d06x5               g049(.a(\b[9] ), .b(\a[10] ), .o1(new_n145));
  nano32aa1n03x7               g050(.a(new_n126), .b(new_n137), .c(new_n144), .d(new_n145), .out0(new_n146));
  nanb03aa1d24x5               g051(.a(new_n138), .b(new_n139), .c(new_n132), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n146), .b(new_n148), .o1(new_n149));
  oai022aa1d24x5               g054(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n150));
  nand23aa1n09x5               g055(.a(new_n150), .b(new_n145), .c(new_n132), .o1(new_n151));
  nor042aa1n03x5               g056(.a(new_n138), .b(new_n131), .o1(new_n152));
  aoi022aa1d18x5               g057(.a(new_n151), .b(new_n152), .c(\b[11] ), .d(\a[12] ), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n149), .c(new_n120), .d(new_n125), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand02aa1n12x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanb02aa1d24x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n155), .c(new_n162), .o1(new_n163));
  xnrc02aa1n02x5               g068(.a(new_n163), .b(new_n160), .out0(\s[14] ));
  nanb02aa1n18x5               g069(.a(new_n161), .b(new_n162), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  oa0012aa1n02x5               g071(.a(new_n158), .b(new_n157), .c(new_n161), .o(new_n167));
  aoi013aa1n06x4               g072(.a(new_n167), .b(new_n155), .c(new_n166), .d(new_n160), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  inv000aa1n02x5               g074(.a(new_n169), .o1(new_n170));
  nand02aa1d24x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n168), .b(new_n171), .c(new_n170), .out0(\s[15] ));
  nona22aa1n02x4               g077(.a(new_n155), .b(new_n165), .c(new_n159), .out0(new_n173));
  nanb02aa1n02x5               g078(.a(new_n167), .b(new_n173), .out0(new_n174));
  nor042aa1d18x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand02aa1d24x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n15x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoi112aa1n02x5               g082(.a(new_n169), .b(new_n177), .c(new_n174), .d(new_n171), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n171), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n177), .o1(new_n180));
  oaoi13aa1n02x5               g085(.a(new_n180), .b(new_n170), .c(new_n168), .d(new_n179), .o1(new_n181));
  norp02aa1n03x5               g086(.a(new_n181), .b(new_n178), .o1(\s[16] ));
  nor043aa1d12x5               g087(.a(new_n165), .b(new_n159), .c(new_n169), .o1(new_n183));
  nanb03aa1n02x5               g088(.a(new_n175), .b(new_n176), .c(new_n171), .out0(new_n184));
  nona23aa1d18x5               g089(.a(new_n183), .b(new_n146), .c(new_n184), .d(new_n147), .out0(new_n185));
  nona23aa1n02x4               g090(.a(new_n158), .b(new_n162), .c(new_n161), .d(new_n157), .out0(new_n186));
  nano32aa1n06x5               g091(.a(new_n186), .b(new_n177), .c(new_n171), .d(new_n170), .out0(new_n187));
  oai112aa1n06x5               g092(.a(new_n158), .b(new_n171), .c(new_n157), .d(new_n161), .o1(new_n188));
  nor042aa1n03x5               g093(.a(new_n175), .b(new_n169), .o1(new_n189));
  aoi022aa1d18x5               g094(.a(new_n188), .b(new_n189), .c(\b[15] ), .d(\a[16] ), .o1(new_n190));
  aoi012aa1n12x5               g095(.a(new_n190), .b(new_n187), .c(new_n153), .o1(new_n191));
  aoai13aa1n12x5               g096(.a(new_n191), .b(new_n185), .c(new_n120), .d(new_n125), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  orn002aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .o(new_n194));
  nanp02aa1n02x5               g099(.a(new_n120), .b(new_n125), .o1(new_n195));
  inv000aa1n02x5               g100(.a(new_n185), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n190), .o1(new_n197));
  aob012aa1n02x5               g102(.a(new_n197), .b(new_n187), .c(new_n153), .out0(new_n198));
  tech160nm_fixorc02aa1n03p5x5 g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n198), .c(new_n195), .d(new_n196), .o1(new_n200));
  xnrc02aa1n12x5               g105(.a(\b[17] ), .b(\a[18] ), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n200), .c(new_n194), .out0(\s[18] ));
  nanp02aa1n02x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  oai022aa1n04x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n205), .b(new_n204), .o1(new_n206));
  nand23aa1n03x5               g111(.a(new_n192), .b(new_n199), .c(new_n202), .o1(new_n207));
  nor002aa1d32x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand42aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  xobna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g117(.a(new_n208), .o1(new_n213));
  aoi012aa1n03x5               g118(.a(new_n210), .b(new_n207), .c(new_n206), .o1(new_n214));
  tech160nm_fixnrc02aa1n04x5   g119(.a(\b[19] ), .b(\a[20] ), .out0(new_n215));
  nano22aa1n03x7               g120(.a(new_n214), .b(new_n213), .c(new_n215), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n206), .o1(new_n217));
  aoi013aa1n02x4               g122(.a(new_n217), .b(new_n192), .c(new_n199), .d(new_n202), .o1(new_n218));
  oaoi13aa1n02x5               g123(.a(new_n215), .b(new_n213), .c(new_n218), .d(new_n210), .o1(new_n219));
  nor002aa1n02x5               g124(.a(new_n219), .b(new_n216), .o1(\s[20] ));
  norb02aa1n03x5               g125(.a(new_n209), .b(new_n215), .out0(new_n221));
  nona23aa1d18x5               g126(.a(new_n221), .b(new_n199), .c(new_n201), .d(new_n208), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nand03aa1n02x5               g128(.a(new_n205), .b(new_n204), .c(new_n209), .o1(new_n224));
  oab012aa1d15x5               g129(.a(new_n208), .b(\a[20] ), .c(\b[19] ), .out0(new_n225));
  aoi022aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(\b[19] ), .d(\a[20] ), .o1(new_n226));
  nor002aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nanp02aa1n03x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n226), .c(new_n192), .d(new_n223), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n226), .c(new_n192), .d(new_n223), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(\s[21] ));
  inv000aa1d42x5               g137(.a(new_n227), .o1(new_n233));
  norp02aa1n04x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nand02aa1d04x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n230), .c(new_n233), .out0(\s[22] ));
  nona23aa1d18x5               g142(.a(new_n235), .b(new_n228), .c(new_n227), .d(new_n234), .out0(new_n238));
  inv040aa1n03x5               g143(.a(new_n238), .o1(new_n239));
  and002aa1n02x5               g144(.a(\b[19] ), .b(\a[20] ), .o(new_n240));
  inv000aa1n06x5               g145(.a(new_n225), .o1(new_n241));
  aoi013aa1n06x4               g146(.a(new_n241), .b(new_n205), .c(new_n209), .d(new_n204), .o1(new_n242));
  aoi012aa1n09x5               g147(.a(new_n234), .b(new_n227), .c(new_n235), .o1(new_n243));
  oai013aa1n09x5               g148(.a(new_n243), .b(new_n242), .c(new_n240), .d(new_n238), .o1(new_n244));
  aoi013aa1n06x4               g149(.a(new_n244), .b(new_n192), .c(new_n223), .d(new_n239), .o1(new_n245));
  xnrb03aa1n03x5               g150(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1n03x5               g152(.a(new_n247), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n192), .b(new_n222), .c(new_n238), .out0(new_n249));
  xnrc02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .out0(new_n250));
  aoib12aa1n03x5               g155(.a(new_n250), .b(new_n249), .c(new_n244), .out0(new_n251));
  xnrc02aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .out0(new_n252));
  nano22aa1n03x7               g157(.a(new_n251), .b(new_n248), .c(new_n252), .out0(new_n253));
  oaoi13aa1n02x5               g158(.a(new_n252), .b(new_n248), .c(new_n245), .d(new_n250), .o1(new_n254));
  nor002aa1n02x5               g159(.a(new_n254), .b(new_n253), .o1(\s[24] ));
  nor042aa1n03x5               g160(.a(new_n252), .b(new_n250), .o1(new_n256));
  nano22aa1n03x7               g161(.a(new_n222), .b(new_n239), .c(new_n256), .out0(new_n257));
  aoi112aa1n03x5               g162(.a(new_n238), .b(new_n240), .c(new_n224), .d(new_n225), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n243), .o1(new_n259));
  oai012aa1n03x5               g164(.a(new_n256), .b(new_n258), .c(new_n259), .o1(new_n260));
  oaoi03aa1n12x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n192), .d(new_n257), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n263), .c(new_n192), .d(new_n257), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  orn002aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .o(new_n268));
  tech160nm_fixnrc02aa1n04x5   g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  xobna2aa1n03x5               g174(.a(new_n269), .b(new_n265), .c(new_n268), .out0(\s[26] ));
  norb02aa1n03x5               g175(.a(new_n264), .b(new_n269), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  tech160nm_fiaoi012aa1n04x5   g177(.a(new_n272), .b(new_n260), .c(new_n262), .o1(new_n273));
  nano32aa1n03x7               g178(.a(new_n222), .b(new_n271), .c(new_n239), .d(new_n256), .out0(new_n274));
  oaoi03aa1n12x5               g179(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .o1(new_n275));
  aoi112aa1n06x5               g180(.a(new_n273), .b(new_n275), .c(new_n192), .d(new_n274), .o1(new_n276));
  xnrb03aa1n03x5               g181(.a(new_n276), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  aoai13aa1n06x5               g182(.a(new_n271), .b(new_n261), .c(new_n244), .d(new_n256), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n274), .b(new_n198), .c(new_n195), .d(new_n196), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n275), .o1(new_n280));
  nanp03aa1n03x5               g185(.a(new_n279), .b(new_n278), .c(new_n280), .o1(new_n281));
  nor042aa1n03x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  tech160nm_fixorc02aa1n05x5   g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  xorc02aa1n12x5               g188(.a(\a[28] ), .b(\b[27] ), .out0(new_n284));
  aoi112aa1n03x4               g189(.a(new_n282), .b(new_n284), .c(new_n281), .d(new_n283), .o1(new_n285));
  inv000aa1n03x5               g190(.a(new_n282), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n283), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n284), .o1(new_n288));
  oaoi13aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n276), .d(new_n287), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n285), .o1(\s[28] ));
  and002aa1n02x5               g195(.a(new_n284), .b(new_n283), .o(new_n291));
  inv000aa1n02x5               g196(.a(new_n291), .o1(new_n292));
  oao003aa1n03x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .carry(new_n293));
  xorc02aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .out0(new_n294));
  inv040aa1n03x5               g199(.a(new_n294), .o1(new_n295));
  oaoi13aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n276), .d(new_n292), .o1(new_n296));
  aoi013aa1n02x4               g201(.a(new_n292), .b(new_n279), .c(new_n278), .d(new_n280), .o1(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g205(.a(new_n295), .b(new_n283), .c(new_n284), .out0(new_n301));
  inv000aa1n02x5               g206(.a(new_n301), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n303));
  tech160nm_fixorc02aa1n04x5   g208(.a(\a[30] ), .b(\b[29] ), .out0(new_n304));
  inv000aa1d42x5               g209(.a(new_n304), .o1(new_n305));
  oaoi13aa1n03x5               g210(.a(new_n305), .b(new_n303), .c(new_n276), .d(new_n302), .o1(new_n306));
  aoi013aa1n02x4               g211(.a(new_n302), .b(new_n279), .c(new_n278), .d(new_n280), .o1(new_n307));
  nano22aa1n02x4               g212(.a(new_n307), .b(new_n303), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano32aa1n02x4               g215(.a(new_n287), .b(new_n304), .c(new_n284), .d(new_n294), .out0(new_n311));
  inv000aa1n02x5               g216(.a(new_n311), .o1(new_n312));
  tech160nm_fioaoi03aa1n03p5x5 g217(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  oaoi13aa1n03x5               g219(.a(new_n310), .b(new_n314), .c(new_n276), .d(new_n312), .o1(new_n315));
  aoi013aa1n02x4               g220(.a(new_n312), .b(new_n279), .c(new_n278), .d(new_n280), .o1(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n310), .c(new_n314), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n315), .b(new_n317), .o1(\s[31] ));
  xobna2aa1n03x5               g223(.a(new_n102), .b(new_n106), .c(new_n104), .out0(\s[3] ));
  xorc02aa1n02x5               g224(.a(\a[4] ), .b(\b[3] ), .out0(new_n320));
  aoi022aa1n02x5               g225(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n321));
  oaoi13aa1n02x5               g226(.a(new_n99), .b(new_n101), .c(new_n321), .d(new_n103), .o1(new_n322));
  mtn022aa1n02x5               g227(.a(new_n107), .b(new_n322), .sa(new_n320), .o1(\s[4] ));
  xorc02aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .out0(new_n324));
  xobna2aa1n03x5               g229(.a(new_n324), .b(new_n107), .c(new_n116), .out0(\s[5] ));
  nanp03aa1n02x5               g230(.a(new_n107), .b(new_n324), .c(new_n116), .o1(new_n326));
  nanb02aa1n02x5               g231(.a(new_n108), .b(new_n109), .out0(new_n327));
  xobna2aa1n03x5               g232(.a(new_n327), .b(new_n326), .c(new_n114), .out0(\s[6] ));
  aoai13aa1n02x5               g233(.a(new_n123), .b(new_n327), .c(new_n326), .d(new_n114), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norp02aa1n02x5               g235(.a(\b[6] ), .b(\a[7] ), .o1(new_n331));
  aoi012aa1n02x5               g236(.a(new_n331), .b(new_n329), .c(new_n117), .o1(new_n332));
  xnrb03aa1n02x5               g237(.a(new_n332), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g238(.a(new_n126), .b(new_n120), .c(new_n125), .out0(\s[9] ));
endmodule


