// Benchmark "adder" written by ABC on Thu Jul 18 08:43:28 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n344, new_n345,
    new_n348, new_n350, new_n351, new_n353;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n04x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n03x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  aoi012aa1n09x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  inv020aa1n04x5               g010(.a(new_n105), .o1(new_n106));
  nor022aa1n16x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nand02aa1n06x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nor022aa1n16x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nand42aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nano23aa1n03x7               g015(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n111));
  inv000aa1d42x5               g016(.a(\a[3] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[2] ), .o1(new_n113));
  aoai13aa1n06x5               g018(.a(new_n108), .b(new_n107), .c(new_n112), .d(new_n113), .o1(new_n114));
  aobi12aa1n06x5               g019(.a(new_n114), .b(new_n111), .c(new_n106), .out0(new_n115));
  nor022aa1n16x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand42aa1d28x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n12x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  xnrc02aa1n03x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  nor002aa1d32x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand02aa1n04x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  norb02aa1n06x5               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  nor002aa1d32x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  nanp02aa1n04x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  norb02aa1n06x5               g029(.a(new_n124), .b(new_n123), .out0(new_n125));
  nona23aa1n03x5               g030(.a(new_n122), .b(new_n125), .c(new_n119), .d(new_n118), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n120), .o1(new_n127));
  aoi112aa1n03x5               g032(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n128));
  inv000aa1n02x5               g033(.a(new_n128), .o1(new_n129));
  nona23aa1n09x5               g034(.a(new_n124), .b(new_n121), .c(new_n120), .d(new_n123), .out0(new_n130));
  oai022aa1n02x5               g035(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n131));
  nano22aa1n06x5               g036(.a(new_n130), .b(new_n117), .c(new_n131), .out0(new_n132));
  nano22aa1n03x7               g037(.a(new_n132), .b(new_n127), .c(new_n129), .out0(new_n133));
  tech160nm_fioai012aa1n03p5x5 g038(.a(new_n133), .b(new_n126), .c(new_n115), .o1(new_n134));
  oaoi03aa1n02x5               g039(.a(new_n100), .b(new_n101), .c(new_n134), .o1(new_n135));
  xnrc02aa1n02x5               g040(.a(new_n135), .b(new_n99), .out0(\s[10] ));
  xorc02aa1n12x5               g041(.a(\a[9] ), .b(\b[8] ), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  oaoi13aa1n06x5               g043(.a(new_n138), .b(new_n133), .c(new_n126), .d(new_n115), .o1(new_n139));
  aoai13aa1n06x5               g044(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n140));
  inv000aa1n03x5               g045(.a(new_n140), .o1(new_n141));
  nor022aa1n08x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nand02aa1n08x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  aoai13aa1n03x5               g049(.a(new_n144), .b(new_n141), .c(new_n139), .d(new_n99), .o1(new_n145));
  aoi113aa1n02x5               g050(.a(new_n144), .b(new_n141), .c(new_n134), .d(new_n137), .e(new_n99), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[11] ));
  nor002aa1d32x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand02aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(new_n150));
  oaoi13aa1n06x5               g055(.a(new_n150), .b(new_n145), .c(\a[11] ), .d(\b[10] ), .o1(new_n151));
  oai112aa1n02x5               g056(.a(new_n145), .b(new_n150), .c(\b[10] ), .d(\a[11] ), .o1(new_n152));
  norb02aa1n03x4               g057(.a(new_n152), .b(new_n151), .out0(\s[12] ));
  nona23aa1n03x5               g058(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n154));
  oaih12aa1n06x5               g059(.a(new_n114), .b(new_n154), .c(new_n105), .o1(new_n155));
  nor043aa1n03x5               g060(.a(new_n130), .b(new_n119), .c(new_n118), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n117), .o1(new_n157));
  nor002aa1n02x5               g062(.a(\b[4] ), .b(\a[5] ), .o1(new_n158));
  norp02aa1n02x5               g063(.a(new_n158), .b(new_n116), .o1(new_n159));
  nona23aa1n06x5               g064(.a(new_n125), .b(new_n122), .c(new_n159), .d(new_n157), .out0(new_n160));
  nona22aa1n06x5               g065(.a(new_n160), .b(new_n128), .c(new_n120), .out0(new_n161));
  nona23aa1n06x5               g066(.a(new_n149), .b(new_n143), .c(new_n142), .d(new_n148), .out0(new_n162));
  nano22aa1n02x4               g067(.a(new_n162), .b(new_n137), .c(new_n99), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n161), .c(new_n155), .d(new_n156), .o1(new_n164));
  nano23aa1n06x5               g069(.a(new_n142), .b(new_n148), .c(new_n149), .d(new_n143), .out0(new_n165));
  aoi112aa1n02x5               g070(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n166));
  aoi112aa1n03x5               g071(.a(new_n166), .b(new_n148), .c(new_n165), .d(new_n141), .o1(new_n167));
  nanp02aa1n03x5               g072(.a(new_n164), .b(new_n167), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  nand42aa1d28x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  tech160nm_fiaoi012aa1n05x5   g076(.a(new_n170), .b(new_n168), .c(new_n171), .o1(new_n172));
  xnrb03aa1n03x5               g077(.a(new_n172), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1n16x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nano23aa1d15x5               g080(.a(new_n170), .b(new_n174), .c(new_n175), .d(new_n171), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  tech160nm_fioai012aa1n03p5x5 g082(.a(new_n175), .b(new_n174), .c(new_n170), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n177), .c(new_n164), .d(new_n167), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1d18x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  nand22aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n06x5               g088(.a(new_n183), .b(new_n181), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n179), .b(new_n184), .o1(new_n185));
  nor042aa1n09x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nand02aa1n16x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  norb02aa1n15x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  aoi012aa1n02x5               g094(.a(new_n189), .b(new_n185), .c(new_n182), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n181), .b(new_n188), .c(new_n179), .d(new_n184), .o1(new_n191));
  norp02aa1n02x5               g096(.a(new_n190), .b(new_n191), .o1(\s[16] ));
  nand23aa1d12x5               g097(.a(new_n176), .b(new_n184), .c(new_n188), .o1(new_n193));
  nano32aa1d12x5               g098(.a(new_n193), .b(new_n165), .c(new_n137), .d(new_n99), .out0(new_n194));
  aoai13aa1n12x5               g099(.a(new_n194), .b(new_n161), .c(new_n155), .d(new_n156), .o1(new_n195));
  inv000aa1d42x5               g100(.a(new_n148), .o1(new_n196));
  inv000aa1n02x5               g101(.a(new_n166), .o1(new_n197));
  oai112aa1n06x5               g102(.a(new_n197), .b(new_n196), .c(new_n162), .d(new_n140), .o1(new_n198));
  inv020aa1n02x5               g103(.a(new_n193), .o1(new_n199));
  nona23aa1n09x5               g104(.a(new_n187), .b(new_n183), .c(new_n181), .d(new_n186), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n181), .b(new_n187), .o1(new_n201));
  oai122aa1n06x5               g106(.a(new_n201), .b(new_n200), .c(new_n178), .d(\b[15] ), .e(\a[16] ), .o1(new_n202));
  aoi012aa1n12x5               g107(.a(new_n202), .b(new_n198), .c(new_n199), .o1(new_n203));
  xorc02aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n195), .c(new_n203), .out0(\s[17] ));
  inv040aa1d32x5               g110(.a(\a[17] ), .o1(new_n206));
  inv030aa1d32x5               g111(.a(\b[16] ), .o1(new_n207));
  nand42aa1n02x5               g112(.a(new_n207), .b(new_n206), .o1(new_n208));
  nor022aa1n02x5               g113(.a(new_n200), .b(new_n178), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n209), .b(new_n186), .c(new_n187), .d(new_n181), .o1(new_n210));
  oai012aa1n03x5               g115(.a(new_n210), .b(new_n167), .c(new_n193), .o1(new_n211));
  aoai13aa1n02x5               g116(.a(new_n204), .b(new_n211), .c(new_n134), .d(new_n194), .o1(new_n212));
  nor042aa1n06x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nand02aa1n06x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nanb02aa1d24x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  xobna2aa1n03x5               g120(.a(new_n215), .b(new_n212), .c(new_n208), .out0(\s[18] ));
  nanp02aa1n02x5               g121(.a(\b[16] ), .b(\a[17] ), .o1(new_n217));
  nano22aa1d15x5               g122(.a(new_n215), .b(new_n208), .c(new_n217), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n214), .b(new_n213), .c(new_n206), .d(new_n207), .o1(new_n220));
  aoai13aa1n04x5               g125(.a(new_n220), .b(new_n219), .c(new_n195), .d(new_n203), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  nand42aa1n06x5               g130(.a(\b[18] ), .b(\a[19] ), .o1(new_n226));
  nanb02aa1n18x5               g131(.a(new_n224), .b(new_n226), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  nand02aa1n02x5               g133(.a(new_n221), .b(new_n228), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[19] ), .o1(new_n230));
  nanb02aa1n06x5               g135(.a(\a[20] ), .b(new_n230), .out0(new_n231));
  nand42aa1n08x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nand02aa1d08x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  tech160nm_fiaoi012aa1n03p5x5 g138(.a(new_n233), .b(new_n229), .c(new_n225), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n233), .o1(new_n235));
  aoi112aa1n03x4               g140(.a(new_n224), .b(new_n235), .c(new_n221), .d(new_n228), .o1(new_n236));
  nor002aa1n02x5               g141(.a(new_n234), .b(new_n236), .o1(\s[20] ));
  nona22aa1n02x4               g142(.a(new_n218), .b(new_n227), .c(new_n233), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n224), .b(new_n232), .o1(new_n239));
  nor043aa1n03x5               g144(.a(new_n220), .b(new_n227), .c(new_n233), .o1(new_n240));
  nano22aa1n03x7               g145(.a(new_n240), .b(new_n231), .c(new_n239), .out0(new_n241));
  aoai13aa1n04x5               g146(.a(new_n241), .b(new_n238), .c(new_n195), .d(new_n203), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n09x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[20] ), .b(\a[21] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nand02aa1n02x5               g152(.a(new_n242), .b(new_n247), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[21] ), .b(\a[22] ), .out0(new_n249));
  tech160nm_fiaoi012aa1n03p5x5 g154(.a(new_n249), .b(new_n248), .c(new_n245), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n249), .o1(new_n251));
  aoi112aa1n03x4               g156(.a(new_n244), .b(new_n251), .c(new_n242), .d(new_n247), .o1(new_n252));
  norp02aa1n03x5               g157(.a(new_n250), .b(new_n252), .o1(\s[22] ));
  nor022aa1n04x5               g158(.a(\b[19] ), .b(\a[20] ), .o1(new_n254));
  nona23aa1n09x5               g159(.a(new_n232), .b(new_n226), .c(new_n224), .d(new_n254), .out0(new_n255));
  nor042aa1n06x5               g160(.a(new_n249), .b(new_n246), .o1(new_n256));
  nanb03aa1n12x5               g161(.a(new_n255), .b(new_n256), .c(new_n218), .out0(new_n257));
  oai112aa1n03x5               g162(.a(new_n239), .b(new_n231), .c(new_n255), .d(new_n220), .o1(new_n258));
  oaoi03aa1n02x5               g163(.a(\a[22] ), .b(\b[21] ), .c(new_n245), .o1(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n260), .b(new_n257), .c(new_n195), .d(new_n203), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  xorc02aa1n06x5               g169(.a(\a[23] ), .b(\b[22] ), .out0(new_n265));
  nand02aa1n02x5               g170(.a(new_n261), .b(new_n265), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[24] ), .b(\b[23] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fiaoi012aa1n03p5x5 g173(.a(new_n268), .b(new_n266), .c(new_n264), .o1(new_n269));
  aoi112aa1n03x4               g174(.a(new_n263), .b(new_n267), .c(new_n261), .d(new_n265), .o1(new_n270));
  nor042aa1n03x5               g175(.a(new_n269), .b(new_n270), .o1(\s[24] ));
  nanp02aa1n03x5               g176(.a(new_n267), .b(new_n265), .o1(new_n272));
  nona23aa1n02x4               g177(.a(new_n256), .b(new_n218), .c(new_n272), .d(new_n255), .out0(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[22] ), .b(\a[23] ), .out0(new_n274));
  norb02aa1n06x5               g179(.a(new_n267), .b(new_n274), .out0(new_n275));
  norp02aa1n02x5               g180(.a(\b[23] ), .b(\a[24] ), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n277));
  nanp03aa1n06x5               g182(.a(new_n259), .b(new_n265), .c(new_n267), .o1(new_n278));
  nona22aa1n09x5               g183(.a(new_n278), .b(new_n277), .c(new_n276), .out0(new_n279));
  aoi013aa1n03x5               g184(.a(new_n279), .b(new_n258), .c(new_n256), .d(new_n275), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n280), .b(new_n273), .c(new_n195), .d(new_n203), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv040aa1n02x5               g188(.a(new_n283), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  nand02aa1n02x5               g190(.a(new_n281), .b(new_n285), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[26] ), .b(\b[25] ), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  tech160nm_fiaoi012aa1n03p5x5 g193(.a(new_n288), .b(new_n286), .c(new_n284), .o1(new_n289));
  aoi112aa1n03x4               g194(.a(new_n283), .b(new_n287), .c(new_n281), .d(new_n285), .o1(new_n290));
  nor002aa1n02x5               g195(.a(new_n289), .b(new_n290), .o1(\s[26] ));
  and002aa1n12x5               g196(.a(new_n287), .b(new_n285), .o(new_n292));
  nano22aa1n03x7               g197(.a(new_n257), .b(new_n292), .c(new_n275), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n211), .c(new_n134), .d(new_n194), .o1(new_n294));
  nano22aa1n03x5               g199(.a(new_n241), .b(new_n256), .c(new_n275), .out0(new_n295));
  oaoi03aa1n09x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .o1(new_n296));
  oaoi13aa1n09x5               g201(.a(new_n296), .b(new_n292), .c(new_n295), .d(new_n279), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n294), .out0(\s[27] ));
  nor042aa1n03x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  nanp02aa1n06x5               g206(.a(new_n195), .b(new_n203), .o1(new_n302));
  nona32aa1n02x4               g207(.a(new_n258), .b(new_n272), .c(new_n249), .d(new_n246), .out0(new_n303));
  inv040aa1n02x5               g208(.a(new_n279), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n292), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n296), .o1(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n305), .c(new_n303), .d(new_n304), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n298), .b(new_n307), .c(new_n302), .d(new_n293), .o1(new_n308));
  xnrc02aa1n12x5               g213(.a(\b[27] ), .b(\a[28] ), .out0(new_n309));
  tech160nm_fiaoi012aa1n03p5x5 g214(.a(new_n309), .b(new_n308), .c(new_n301), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n298), .o1(new_n311));
  aoi012aa1n02x7               g216(.a(new_n311), .b(new_n297), .c(new_n294), .o1(new_n312));
  nano22aa1n03x5               g217(.a(new_n312), .b(new_n301), .c(new_n309), .out0(new_n313));
  norp02aa1n03x5               g218(.a(new_n310), .b(new_n313), .o1(\s[28] ));
  norb02aa1d21x5               g219(.a(new_n298), .b(new_n309), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n307), .c(new_n302), .d(new_n293), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[28] ), .b(\a[29] ), .out0(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n315), .o1(new_n320));
  aoi012aa1n02x7               g225(.a(new_n320), .b(new_n297), .c(new_n294), .o1(new_n321));
  nano22aa1n02x4               g226(.a(new_n321), .b(new_n317), .c(new_n318), .out0(new_n322));
  norp02aa1n03x5               g227(.a(new_n319), .b(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g228(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n09x5               g229(.a(new_n298), .b(new_n318), .c(new_n309), .out0(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n307), .c(new_n302), .d(new_n293), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .out0(new_n328));
  tech160nm_fiaoi012aa1n03p5x5 g233(.a(new_n328), .b(new_n326), .c(new_n327), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n325), .o1(new_n330));
  aoi012aa1n02x7               g235(.a(new_n330), .b(new_n297), .c(new_n294), .o1(new_n331));
  nano22aa1n02x4               g236(.a(new_n331), .b(new_n327), .c(new_n328), .out0(new_n332));
  norp02aa1n03x5               g237(.a(new_n329), .b(new_n332), .o1(\s[30] ));
  xnrc02aa1n02x5               g238(.a(\b[30] ), .b(\a[31] ), .out0(new_n334));
  norb02aa1n06x5               g239(.a(new_n325), .b(new_n328), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n307), .c(new_n302), .d(new_n293), .o1(new_n336));
  oao003aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(new_n327), .carry(new_n337));
  tech160nm_fiaoi012aa1n02p5x5 g242(.a(new_n334), .b(new_n336), .c(new_n337), .o1(new_n338));
  inv000aa1n02x5               g243(.a(new_n335), .o1(new_n339));
  aoi012aa1n02x7               g244(.a(new_n339), .b(new_n297), .c(new_n294), .o1(new_n340));
  nano22aa1n02x4               g245(.a(new_n340), .b(new_n334), .c(new_n337), .out0(new_n341));
  nor002aa1n02x5               g246(.a(new_n338), .b(new_n341), .o1(\s[31] ));
  xorb03aa1n02x5               g247(.a(new_n105), .b(\b[2] ), .c(new_n112), .out0(\s[3] ));
  nona22aa1n02x4               g248(.a(new_n110), .b(new_n105), .c(new_n109), .out0(new_n344));
  aboi22aa1n03x5               g249(.a(new_n107), .b(new_n108), .c(new_n112), .d(new_n113), .out0(new_n345));
  aboi22aa1n03x5               g250(.a(new_n107), .b(new_n155), .c(new_n344), .d(new_n345), .out0(\s[4] ));
  xorb03aa1n02x5               g251(.a(new_n155), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g252(.a(\a[5] ), .b(\b[4] ), .c(new_n115), .o1(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g254(.a(new_n125), .b(new_n116), .c(new_n348), .d(new_n117), .o1(new_n350));
  aoi112aa1n02x5               g255(.a(new_n125), .b(new_n116), .c(new_n348), .d(new_n117), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n350), .b(new_n351), .out0(\s[7] ));
  inv000aa1d42x5               g257(.a(new_n123), .o1(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n122), .b(new_n350), .c(new_n353), .out0(\s[8] ));
  xorb03aa1n02x5               g259(.a(new_n134), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


