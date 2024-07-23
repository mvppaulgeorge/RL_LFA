// Benchmark "adder" written by ABC on Wed Jul 17 15:05:17 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n228, new_n229, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n345, new_n346, new_n347, new_n349, new_n350, new_n353, new_n354,
    new_n355, new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  nor042aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n06x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n02x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nand23aa1n03x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  tech160nm_fioai012aa1n04x5   g013(.a(new_n103), .b(new_n105), .c(new_n102), .o1(new_n109));
  tech160nm_fixorc02aa1n04x5   g014(.a(\a[5] ), .b(\b[4] ), .out0(new_n110));
  inv000aa1d42x5               g015(.a(\b[5] ), .o1(new_n111));
  nanb02aa1n06x5               g016(.a(\a[6] ), .b(new_n111), .out0(new_n112));
  nand42aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(new_n112), .b(new_n113), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  tech160nm_finand02aa1n03p5x5 g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norb02aa1n06x4               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nor002aa1d32x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand42aa1n03x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nanb02aa1n03x5               g024(.a(new_n118), .b(new_n119), .out0(new_n120));
  nona23aa1n09x5               g025(.a(new_n117), .b(new_n110), .c(new_n120), .d(new_n114), .out0(new_n121));
  nano22aa1n02x4               g026(.a(new_n118), .b(new_n113), .c(new_n119), .out0(new_n122));
  oai112aa1n06x5               g027(.a(new_n112), .b(new_n113), .c(\b[4] ), .d(\a[5] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(new_n118), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  aoi013aa1n03x5               g030(.a(new_n125), .b(new_n122), .c(new_n123), .d(new_n117), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n121), .c(new_n108), .d(new_n109), .o1(new_n127));
  nand02aa1n08x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nor002aa1n04x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n08x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n97), .c(new_n127), .d(new_n128), .o1(new_n132));
  aoi112aa1n02x5               g037(.a(new_n131), .b(new_n97), .c(new_n127), .d(new_n128), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(\s[10] ));
  norb03aa1n06x5               g039(.a(new_n130), .b(new_n97), .c(new_n129), .out0(new_n135));
  oaoi03aa1n02x5               g040(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n136));
  nona23aa1n02x4               g041(.a(new_n106), .b(new_n103), .c(new_n102), .d(new_n105), .out0(new_n137));
  oai012aa1n04x7               g042(.a(new_n109), .b(new_n137), .c(new_n136), .o1(new_n138));
  nona23aa1n02x4               g043(.a(new_n119), .b(new_n116), .c(new_n115), .d(new_n118), .out0(new_n139));
  norb03aa1n03x5               g044(.a(new_n110), .b(new_n139), .c(new_n114), .out0(new_n140));
  nanp03aa1n02x5               g045(.a(new_n122), .b(new_n123), .c(new_n117), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n125), .b(new_n141), .out0(new_n142));
  nano23aa1n06x5               g047(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n128), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n140), .d(new_n138), .o1(new_n144));
  oaib12aa1n02x5               g049(.a(new_n144), .b(new_n135), .c(new_n130), .out0(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n08x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nand42aa1n16x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  xnrc02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n145), .d(new_n148), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(new_n147), .b(new_n149), .c(new_n145), .d(new_n148), .o1(new_n151));
  nanb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(\s[12] ));
  nand02aa1n10x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  oai012aa1d24x5               g058(.a(new_n148), .b(\b[11] ), .c(\a[12] ), .o1(new_n154));
  nano23aa1n06x5               g059(.a(new_n154), .b(new_n147), .c(new_n130), .d(new_n153), .out0(new_n155));
  nand22aa1n09x5               g060(.a(new_n155), .b(new_n143), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n142), .c(new_n140), .d(new_n138), .o1(new_n158));
  aoi012aa1n12x5               g063(.a(new_n147), .b(\a[10] ), .c(\b[9] ), .o1(new_n159));
  nanb03aa1d24x5               g064(.a(new_n154), .b(new_n159), .c(new_n153), .out0(new_n160));
  norp02aa1n02x5               g065(.a(\b[11] ), .b(\a[12] ), .o1(new_n161));
  oaih12aa1n02x5               g066(.a(new_n153), .b(new_n161), .c(new_n147), .o1(new_n162));
  oai012aa1n18x5               g067(.a(new_n162), .b(new_n160), .c(new_n135), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  norp02aa1n04x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand42aa1n08x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n158), .c(new_n164), .out0(\s[13] ));
  aobi12aa1n02x5               g073(.a(new_n167), .b(new_n158), .c(new_n164), .out0(new_n169));
  nor022aa1n08x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n10x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  oai012aa1n02x5               g077(.a(new_n172), .b(new_n169), .c(new_n165), .o1(new_n173));
  norb03aa1n02x5               g078(.a(new_n171), .b(new_n165), .c(new_n170), .out0(new_n174));
  oaib12aa1n02x5               g079(.a(new_n173), .b(new_n169), .c(new_n174), .out0(\s[14] ));
  oai012aa1n02x5               g080(.a(new_n171), .b(new_n170), .c(new_n165), .o1(new_n176));
  nano23aa1n06x5               g081(.a(new_n165), .b(new_n170), .c(new_n171), .d(new_n166), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n163), .c(new_n127), .d(new_n157), .o1(new_n178));
  xorc02aa1n02x5               g083(.a(\a[15] ), .b(\b[14] ), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n178), .c(new_n176), .out0(\s[15] ));
  nona23aa1n02x4               g085(.a(new_n171), .b(new_n166), .c(new_n165), .d(new_n170), .out0(new_n181));
  aoai13aa1n02x7               g086(.a(new_n176), .b(new_n181), .c(new_n158), .d(new_n164), .o1(new_n182));
  inv040aa1d32x5               g087(.a(\a[15] ), .o1(new_n183));
  inv040aa1d28x5               g088(.a(\b[14] ), .o1(new_n184));
  nand02aa1n06x5               g089(.a(new_n184), .b(new_n183), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\a[16] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[15] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  nand42aa1n02x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(new_n189), .b(new_n190), .o1(new_n191));
  aoai13aa1n03x5               g096(.a(new_n191), .b(new_n186), .c(new_n182), .d(new_n179), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n179), .o1(new_n193));
  nano22aa1n02x4               g098(.a(new_n186), .b(new_n189), .c(new_n190), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n178), .d(new_n176), .o1(new_n195));
  nanp02aa1n03x5               g100(.a(new_n192), .b(new_n195), .o1(\s[16] ));
  oai022aa1n02x5               g101(.a(new_n183), .b(new_n184), .c(\b[15] ), .d(\a[16] ), .o1(new_n197));
  nano32aa1n03x7               g102(.a(new_n197), .b(new_n185), .c(new_n190), .d(new_n171), .out0(new_n198));
  nano22aa1d15x5               g103(.a(new_n156), .b(new_n198), .c(new_n177), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n142), .c(new_n140), .d(new_n138), .o1(new_n200));
  aoi022aa1n02x5               g105(.a(new_n184), .b(new_n183), .c(\a[14] ), .d(\b[13] ), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n188), .b(new_n187), .c(\a[15] ), .d(\b[14] ), .o1(new_n202));
  nano32aa1n03x7               g107(.a(new_n181), .b(new_n202), .c(new_n190), .d(new_n201), .out0(new_n203));
  nanp03aa1n02x5               g108(.a(new_n201), .b(new_n202), .c(new_n190), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[16] ), .b(\b[15] ), .c(new_n185), .o1(new_n205));
  oabi12aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n174), .out0(new_n206));
  aoi012aa1n09x5               g111(.a(new_n206), .b(new_n163), .c(new_n203), .o1(new_n207));
  nor002aa1n04x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  and002aa1n12x5               g113(.a(\b[16] ), .b(\a[17] ), .o(new_n209));
  norp02aa1n02x5               g114(.a(new_n209), .b(new_n208), .o1(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n200), .c(new_n207), .out0(\s[17] ));
  nanp02aa1n02x5               g116(.a(new_n200), .b(new_n207), .o1(new_n212));
  inv040aa1d32x5               g117(.a(\a[18] ), .o1(new_n213));
  inv000aa1d48x5               g118(.a(\b[17] ), .o1(new_n214));
  nanp02aa1n24x5               g119(.a(new_n214), .b(new_n213), .o1(new_n215));
  nand02aa1d24x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  nand02aa1d04x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n208), .c(new_n212), .d(new_n210), .o1(new_n218));
  oai112aa1n06x5               g123(.a(new_n215), .b(new_n216), .c(\b[16] ), .d(\a[17] ), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n219), .c(new_n210), .d(new_n212), .o1(\s[18] ));
  nand02aa1d04x5               g125(.a(new_n163), .b(new_n203), .o1(new_n221));
  inv000aa1n02x5               g126(.a(new_n206), .o1(new_n222));
  nand02aa1d06x5               g127(.a(new_n221), .b(new_n222), .o1(new_n223));
  nor043aa1d12x5               g128(.a(new_n217), .b(new_n209), .c(new_n208), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n223), .c(new_n127), .d(new_n199), .o1(new_n225));
  oaoi03aa1n02x5               g130(.a(new_n213), .b(new_n214), .c(new_n208), .o1(new_n226));
  nor042aa1n09x5               g131(.a(\b[18] ), .b(\a[19] ), .o1(new_n227));
  nand02aa1n08x5               g132(.a(\b[18] ), .b(\a[19] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  xnbna2aa1n03x5               g134(.a(new_n229), .b(new_n225), .c(new_n226), .out0(\s[19] ));
  xnrc02aa1n02x5               g135(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x7               g136(.a(new_n229), .b(new_n225), .c(new_n226), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n224), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n226), .b(new_n233), .c(new_n200), .d(new_n207), .o1(new_n234));
  nor042aa1n04x5               g139(.a(\b[19] ), .b(\a[20] ), .o1(new_n235));
  nand42aa1n16x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  nanb02aa1n02x5               g141(.a(new_n235), .b(new_n236), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n227), .c(new_n234), .d(new_n228), .o1(new_n238));
  norb03aa1n02x5               g143(.a(new_n236), .b(new_n227), .c(new_n235), .out0(new_n239));
  oaib12aa1n02x5               g144(.a(new_n238), .b(new_n232), .c(new_n239), .out0(\s[20] ));
  aoi012aa1d24x5               g145(.a(new_n227), .b(\a[18] ), .c(\b[17] ), .o1(new_n241));
  nano22aa1n12x5               g146(.a(new_n235), .b(new_n228), .c(new_n236), .out0(new_n242));
  nand23aa1d12x5               g147(.a(new_n224), .b(new_n241), .c(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n223), .c(new_n127), .d(new_n199), .o1(new_n245));
  nand23aa1n06x5               g150(.a(new_n242), .b(new_n219), .c(new_n241), .o1(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n236), .b(new_n235), .c(new_n227), .o1(new_n247));
  nand02aa1n04x5               g152(.a(new_n246), .b(new_n247), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  nor002aa1n16x5               g154(.a(\b[20] ), .b(\a[21] ), .o1(new_n250));
  nand02aa1d24x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n245), .c(new_n249), .out0(\s[21] ));
  aobi12aa1n02x5               g158(.a(new_n252), .b(new_n245), .c(new_n249), .out0(new_n254));
  aoai13aa1n02x7               g159(.a(new_n249), .b(new_n243), .c(new_n200), .d(new_n207), .o1(new_n255));
  nor022aa1n12x5               g160(.a(\b[21] ), .b(\a[22] ), .o1(new_n256));
  nand42aa1d28x5               g161(.a(\b[21] ), .b(\a[22] ), .o1(new_n257));
  nanb02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n250), .c(new_n255), .d(new_n252), .o1(new_n259));
  nona22aa1n09x5               g164(.a(new_n257), .b(new_n256), .c(new_n250), .out0(new_n260));
  oaih12aa1n02x5               g165(.a(new_n259), .b(new_n260), .c(new_n254), .o1(\s[22] ));
  and002aa1n02x5               g166(.a(new_n242), .b(new_n241), .o(new_n262));
  nano23aa1d15x5               g167(.a(new_n250), .b(new_n256), .c(new_n257), .d(new_n251), .out0(new_n263));
  nand23aa1n02x5               g168(.a(new_n262), .b(new_n224), .c(new_n263), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n223), .c(new_n127), .d(new_n199), .o1(new_n266));
  aoi022aa1n02x5               g171(.a(new_n248), .b(new_n263), .c(new_n257), .d(new_n260), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n264), .c(new_n200), .d(new_n207), .o1(new_n268));
  norp02aa1n02x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  nand02aa1d08x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n270), .b(new_n269), .out0(new_n271));
  aoi122aa1n02x5               g176(.a(new_n271), .b(new_n257), .c(new_n260), .d(new_n248), .e(new_n263), .o1(new_n272));
  aoi022aa1n02x5               g177(.a(new_n268), .b(new_n271), .c(new_n266), .d(new_n272), .o1(\s[23] ));
  aobi12aa1n02x5               g178(.a(new_n271), .b(new_n266), .c(new_n267), .out0(new_n274));
  orn002aa1n24x5               g179(.a(\a[24] ), .b(\b[23] ), .o(new_n275));
  nanp02aa1n12x5               g180(.a(\b[23] ), .b(\a[24] ), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(new_n275), .b(new_n276), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n269), .c(new_n268), .d(new_n270), .o1(new_n278));
  oai112aa1n06x5               g183(.a(new_n275), .b(new_n276), .c(\b[22] ), .d(\a[23] ), .o1(new_n279));
  oai012aa1n02x5               g184(.a(new_n278), .b(new_n279), .c(new_n274), .o1(\s[24] ));
  inv000aa1n04x5               g185(.a(new_n276), .o1(new_n281));
  oai012aa1n06x5               g186(.a(new_n257), .b(\b[22] ), .c(\a[23] ), .o1(new_n282));
  oai012aa1n06x5               g187(.a(new_n270), .b(\b[23] ), .c(\a[24] ), .o1(new_n283));
  norp03aa1n09x5               g188(.a(new_n283), .b(new_n282), .c(new_n281), .o1(new_n284));
  nand22aa1n03x5               g189(.a(new_n284), .b(new_n263), .o1(new_n285));
  nano32aa1n03x7               g190(.a(new_n285), .b(new_n224), .c(new_n241), .d(new_n242), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n223), .c(new_n127), .d(new_n199), .o1(new_n287));
  aoi022aa1n06x5               g192(.a(new_n284), .b(new_n260), .c(new_n276), .d(new_n279), .o1(new_n288));
  aoai13aa1n12x5               g193(.a(new_n288), .b(new_n285), .c(new_n246), .d(new_n247), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  tech160nm_fixorc02aa1n03p5x5 g195(.a(\a[25] ), .b(\b[24] ), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n287), .c(new_n290), .out0(\s[25] ));
  aobi12aa1n02x5               g197(.a(new_n291), .b(new_n287), .c(new_n290), .out0(new_n293));
  inv000aa1n02x5               g198(.a(new_n286), .o1(new_n294));
  aoai13aa1n02x5               g199(.a(new_n290), .b(new_n294), .c(new_n200), .d(new_n207), .o1(new_n295));
  norp02aa1n02x5               g200(.a(\b[24] ), .b(\a[25] ), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[25] ), .b(\a[26] ), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n295), .d(new_n291), .o1(new_n298));
  oabi12aa1n02x5               g203(.a(new_n297), .b(\a[25] ), .c(\b[24] ), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n298), .b(new_n293), .c(new_n299), .o1(\s[26] ));
  norb02aa1n02x5               g205(.a(new_n291), .b(new_n297), .out0(new_n301));
  nano32aa1n03x7               g206(.a(new_n243), .b(new_n301), .c(new_n263), .d(new_n284), .out0(new_n302));
  inv020aa1n03x5               g207(.a(new_n302), .o1(new_n303));
  nanp02aa1n02x5               g208(.a(\b[25] ), .b(\a[26] ), .o1(new_n304));
  aoi022aa1n12x5               g209(.a(new_n289), .b(new_n301), .c(new_n304), .d(new_n299), .o1(new_n305));
  aoai13aa1n06x5               g210(.a(new_n305), .b(new_n303), .c(new_n200), .d(new_n207), .o1(new_n306));
  xorb03aa1n03x5               g211(.a(new_n306), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  xorc02aa1n12x5               g213(.a(\a[27] ), .b(\b[26] ), .out0(new_n309));
  xnrc02aa1n12x5               g214(.a(\b[27] ), .b(\a[28] ), .out0(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n308), .c(new_n306), .d(new_n309), .o1(new_n311));
  aoai13aa1n04x5               g216(.a(new_n302), .b(new_n223), .c(new_n127), .d(new_n199), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n309), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n310), .b(new_n308), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n313), .c(new_n312), .d(new_n305), .o1(new_n315));
  nanp02aa1n03x5               g220(.a(new_n311), .b(new_n315), .o1(\s[28] ));
  xorc02aa1n12x5               g221(.a(\a[29] ), .b(\b[28] ), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  norb02aa1n12x5               g223(.a(new_n309), .b(new_n310), .out0(new_n319));
  tech160nm_fiaoi012aa1n05x5   g224(.a(new_n314), .b(\a[28] ), .c(\b[27] ), .o1(new_n320));
  aoai13aa1n02x7               g225(.a(new_n318), .b(new_n320), .c(new_n306), .d(new_n319), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n319), .o1(new_n322));
  norp02aa1n02x5               g227(.a(new_n320), .b(new_n318), .o1(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n322), .c(new_n312), .d(new_n305), .o1(new_n324));
  nanp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(\s[29] ));
  xorb03aa1n02x5               g230(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g231(.a(new_n310), .b(new_n309), .c(new_n317), .out0(new_n327));
  nand42aa1n02x5               g232(.a(new_n306), .b(new_n327), .o1(new_n328));
  norp02aa1n02x5               g233(.a(\b[28] ), .b(\a[29] ), .o1(new_n329));
  aoi012aa1n02x5               g234(.a(new_n329), .b(new_n320), .c(new_n317), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .out0(new_n331));
  inv000aa1n02x5               g236(.a(new_n327), .o1(new_n332));
  oai012aa1n02x5               g237(.a(new_n331), .b(\b[28] ), .c(\a[29] ), .o1(new_n333));
  aoi012aa1n02x5               g238(.a(new_n333), .b(new_n320), .c(new_n317), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n332), .c(new_n312), .d(new_n305), .o1(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n331), .c(new_n328), .d(new_n330), .o1(\s[30] ));
  nano23aa1n06x5               g241(.a(new_n318), .b(new_n310), .c(new_n331), .d(new_n309), .out0(new_n337));
  aoi012aa1n02x7               g242(.a(new_n334), .b(\a[30] ), .c(\b[29] ), .o1(new_n338));
  xnrc02aa1n02x5               g243(.a(\b[30] ), .b(\a[31] ), .out0(new_n339));
  aoai13aa1n02x7               g244(.a(new_n339), .b(new_n338), .c(new_n306), .d(new_n337), .o1(new_n340));
  inv000aa1n02x5               g245(.a(new_n337), .o1(new_n341));
  norp02aa1n02x5               g246(.a(new_n338), .b(new_n339), .o1(new_n342));
  aoai13aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(new_n312), .d(new_n305), .o1(new_n343));
  nanp02aa1n03x5               g248(.a(new_n340), .b(new_n343), .o1(\s[31] ));
  aoi022aa1n02x5               g249(.a(new_n99), .b(new_n98), .c(\a[1] ), .d(\b[0] ), .o1(new_n345));
  oaib12aa1n02x5               g250(.a(new_n345), .b(new_n99), .c(\a[2] ), .out0(new_n346));
  aboi22aa1n03x5               g251(.a(new_n105), .b(new_n106), .c(new_n98), .d(new_n99), .out0(new_n347));
  aoi022aa1n02x5               g252(.a(new_n101), .b(new_n107), .c(new_n346), .d(new_n347), .o1(\s[3] ));
  inv000aa1d42x5               g253(.a(new_n105), .o1(new_n349));
  nanp02aa1n02x5               g254(.a(new_n101), .b(new_n107), .o1(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n104), .b(new_n350), .c(new_n349), .out0(\s[4] ));
  xnbna2aa1n03x5               g256(.a(new_n110), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  nanp02aa1n02x5               g257(.a(new_n138), .b(new_n110), .o1(new_n353));
  oa0012aa1n02x5               g258(.a(new_n353), .b(\b[4] ), .c(\a[5] ), .o(new_n354));
  nanb02aa1n02x5               g259(.a(new_n123), .b(new_n353), .out0(new_n355));
  aoai13aa1n02x5               g260(.a(new_n355), .b(new_n354), .c(new_n112), .d(new_n113), .o1(\s[6] ));
  xnbna2aa1n03x5               g261(.a(new_n120), .b(new_n355), .c(new_n113), .out0(\s[7] ));
  aoai13aa1n02x5               g262(.a(new_n122), .b(new_n123), .c(new_n138), .d(new_n110), .o1(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n117), .b(new_n358), .c(new_n124), .out0(\s[8] ));
  xorb03aa1n02x5               g264(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


