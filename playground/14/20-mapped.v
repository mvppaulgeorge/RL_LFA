// Benchmark "adder" written by ABC on Wed Jul 17 19:17:45 2024

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
    new_n170, new_n171, new_n173, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n297, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n303, new_n304, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n329, new_n330, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n336, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n345, new_n346, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n355, new_n358, new_n359,
    new_n361, new_n363, new_n364, new_n366, new_n367, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n12x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n03x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanb02aa1n02x5               g005(.a(new_n99), .b(new_n100), .out0(new_n101));
  orn002aa1n02x5               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nanp02aa1n04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n02x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  nanp02aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb03aa1n03x5               g011(.a(new_n105), .b(new_n99), .c(new_n106), .out0(new_n107));
  aoai13aa1n06x5               g012(.a(new_n107), .b(new_n101), .c(new_n104), .d(new_n102), .o1(new_n108));
  nand42aa1d28x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  aoi022aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n110));
  oai112aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(\b[7] ), .d(\a[8] ), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n105), .b(\b[6] ), .c(\a[7] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[4] ), .o1(new_n114));
  nanb02aa1n12x5               g019(.a(\a[5] ), .b(new_n114), .out0(new_n115));
  oai112aa1n02x5               g020(.a(new_n115), .b(new_n113), .c(\b[5] ), .d(\a[6] ), .o1(new_n116));
  nor043aa1n03x5               g021(.a(new_n111), .b(new_n116), .c(new_n112), .o1(new_n117));
  nand42aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nand02aa1d16x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand23aa1n09x5               g024(.a(new_n118), .b(new_n109), .c(new_n119), .o1(new_n120));
  oaih22aa1n04x5               g025(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n118), .o1(new_n122));
  nor002aa1n04x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nor022aa1n08x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  norb03aa1n09x5               g029(.a(new_n109), .b(new_n124), .c(new_n123), .out0(new_n125));
  oai013aa1n03x5               g030(.a(new_n122), .b(new_n125), .c(new_n120), .d(new_n121), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(new_n128));
  aoai13aa1n03x5               g033(.a(new_n128), .b(new_n126), .c(new_n117), .d(new_n108), .o1(new_n129));
  nor042aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  tech160nm_finand02aa1n05x5   g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  norb02aa1n02x5               g038(.a(new_n100), .b(new_n99), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n104), .b(new_n102), .o1(new_n135));
  aobi12aa1n02x5               g040(.a(new_n107), .b(new_n135), .c(new_n134), .out0(new_n136));
  norp02aa1n02x5               g041(.a(\b[7] ), .b(\a[8] ), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n120), .o1(new_n138));
  norb03aa1n02x5               g043(.a(new_n113), .b(new_n123), .c(new_n124), .out0(new_n139));
  nona23aa1n02x4               g044(.a(new_n138), .b(new_n139), .c(new_n112), .d(new_n137), .out0(new_n140));
  norp03aa1n02x5               g045(.a(new_n125), .b(new_n121), .c(new_n120), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n122), .b(new_n141), .out0(new_n142));
  oai012aa1n02x5               g047(.a(new_n142), .b(new_n136), .c(new_n140), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n132), .b(new_n97), .c(new_n143), .d(new_n127), .o1(new_n144));
  oai012aa1d24x5               g049(.a(new_n131), .b(new_n130), .c(new_n97), .o1(new_n145));
  nor042aa1n09x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand42aa1n03x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norb02aa1n15x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n145), .out0(\s[11] ));
  inv000aa1d42x5               g054(.a(new_n146), .o1(new_n150));
  nanp02aa1n03x5               g055(.a(new_n129), .b(new_n98), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n145), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n148), .b(new_n152), .c(new_n151), .d(new_n132), .o1(new_n153));
  nor002aa1n03x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nand42aa1n03x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  norb02aa1n06x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  tech160nm_fiaoi012aa1n03p5x5 g062(.a(new_n157), .b(new_n153), .c(new_n150), .o1(new_n158));
  nona22aa1n02x5               g063(.a(new_n153), .b(new_n156), .c(new_n146), .out0(new_n159));
  norb02aa1n03x4               g064(.a(new_n159), .b(new_n158), .out0(\s[12] ));
  norb03aa1n03x5               g065(.a(new_n131), .b(new_n97), .c(new_n130), .out0(new_n161));
  nano22aa1n02x4               g066(.a(new_n154), .b(new_n127), .c(new_n155), .out0(new_n162));
  and003aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n148), .o(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n126), .c(new_n117), .d(new_n108), .o1(new_n164));
  tech160nm_fioai012aa1n03p5x5 g069(.a(new_n155), .b(new_n154), .c(new_n146), .o1(new_n165));
  nona23aa1n09x5               g070(.a(new_n155), .b(new_n147), .c(new_n146), .d(new_n154), .out0(new_n166));
  oai012aa1d24x5               g071(.a(new_n165), .b(new_n166), .c(new_n145), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  nor042aa1n06x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  and002aa1n09x5               g074(.a(\b[12] ), .b(\a[13] ), .o(new_n170));
  norp02aa1n02x5               g075(.a(new_n170), .b(new_n169), .o1(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n164), .c(new_n168), .out0(\s[13] ));
  inv000aa1d42x5               g077(.a(new_n169), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n170), .c(new_n164), .d(new_n168), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g080(.a(\b[13] ), .b(\a[14] ), .o1(new_n176));
  nand42aa1n02x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  oai012aa1n04x7               g082(.a(new_n177), .b(new_n176), .c(new_n169), .o1(new_n178));
  nano23aa1n02x4               g083(.a(new_n176), .b(new_n170), .c(new_n173), .d(new_n177), .out0(new_n179));
  inv000aa1n02x5               g084(.a(new_n179), .o1(new_n180));
  aoai13aa1n04x5               g085(.a(new_n178), .b(new_n180), .c(new_n164), .d(new_n168), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nanp02aa1n04x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nor042aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  aoai13aa1n04x5               g092(.a(new_n187), .b(new_n183), .c(new_n181), .d(new_n184), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(new_n183), .b(new_n187), .c(new_n181), .d(new_n184), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(\s[16] ));
  aoi012aa1n06x5               g095(.a(new_n126), .b(new_n117), .c(new_n108), .o1(new_n191));
  inv000aa1n02x5               g096(.a(new_n170), .o1(new_n192));
  nona22aa1n03x5               g097(.a(new_n177), .b(new_n176), .c(new_n169), .out0(new_n193));
  inv000aa1n02x5               g098(.a(new_n183), .o1(new_n194));
  nanb03aa1n03x5               g099(.a(new_n185), .b(new_n186), .c(new_n184), .out0(new_n195));
  nano23aa1n06x5               g100(.a(new_n193), .b(new_n195), .c(new_n194), .d(new_n192), .out0(new_n196));
  nand22aa1n03x5               g101(.a(new_n163), .b(new_n196), .o1(new_n197));
  oaoi03aa1n02x5               g102(.a(\a[16] ), .b(\b[15] ), .c(new_n194), .o1(new_n198));
  nona23aa1n02x4               g103(.a(new_n186), .b(new_n184), .c(new_n183), .d(new_n185), .out0(new_n199));
  oabi12aa1n06x5               g104(.a(new_n198), .b(new_n199), .c(new_n178), .out0(new_n200));
  aoi012aa1d18x5               g105(.a(new_n200), .b(new_n167), .c(new_n196), .o1(new_n201));
  oai012aa1n18x5               g106(.a(new_n201), .b(new_n191), .c(new_n197), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norb03aa1n03x5               g108(.a(new_n177), .b(new_n169), .c(new_n176), .out0(new_n204));
  nano22aa1n03x5               g109(.a(new_n185), .b(new_n184), .c(new_n186), .out0(new_n205));
  nona23aa1n03x5               g110(.a(new_n205), .b(new_n204), .c(new_n170), .d(new_n183), .out0(new_n206));
  nano32aa1n03x7               g111(.a(new_n206), .b(new_n162), .c(new_n148), .d(new_n161), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n126), .c(new_n108), .d(new_n117), .o1(new_n208));
  inv040aa1d32x5               g113(.a(\a[17] ), .o1(new_n209));
  inv040aa1d32x5               g114(.a(\b[16] ), .o1(new_n210));
  nand42aa1n03x5               g115(.a(new_n210), .b(new_n209), .o1(new_n211));
  and002aa1n06x5               g116(.a(\b[16] ), .b(\a[17] ), .o(new_n212));
  aoai13aa1n03x5               g117(.a(new_n211), .b(new_n212), .c(new_n208), .d(new_n201), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1d32x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nand42aa1d28x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  nano23aa1d15x5               g121(.a(new_n215), .b(new_n212), .c(new_n211), .d(new_n216), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n12x5               g123(.a(new_n216), .b(new_n215), .c(new_n209), .d(new_n210), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n218), .c(new_n208), .d(new_n201), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n219), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[18] ), .b(\a[19] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n225), .c(new_n202), .d(new_n217), .o1(new_n228));
  inv040aa1d28x5               g133(.a(\a[20] ), .o1(new_n229));
  inv030aa1d32x5               g134(.a(\b[19] ), .o1(new_n230));
  nand02aa1n04x5               g135(.a(new_n230), .b(new_n229), .o1(new_n231));
  nanp02aa1n04x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nand22aa1n09x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  aoi012aa1n03x5               g138(.a(new_n233), .b(new_n228), .c(new_n224), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n233), .o1(new_n235));
  aoi112aa1n03x4               g140(.a(new_n223), .b(new_n235), .c(new_n220), .d(new_n227), .o1(new_n236));
  norp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[20] ));
  and002aa1n02x5               g142(.a(\b[18] ), .b(\a[19] ), .o(new_n238));
  nona32aa1d24x5               g143(.a(new_n217), .b(new_n233), .c(new_n238), .d(new_n223), .out0(new_n239));
  tech160nm_fioaoi03aa1n03p5x5 g144(.a(new_n229), .b(new_n230), .c(new_n223), .o1(new_n240));
  oai013aa1d12x5               g145(.a(new_n240), .b(new_n226), .c(new_n219), .d(new_n233), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n242), .b(new_n239), .c(new_n208), .d(new_n201), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n16x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n239), .o1(new_n247));
  nand42aa1n03x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n245), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n241), .c(new_n202), .d(new_n247), .o1(new_n250));
  nor042aa1n02x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  tech160nm_finand02aa1n03p5x5 g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  tech160nm_fiaoi012aa1n05x5   g159(.a(new_n254), .b(new_n250), .c(new_n246), .o1(new_n255));
  aoi112aa1n02x7               g160(.a(new_n245), .b(new_n253), .c(new_n243), .d(new_n248), .o1(new_n256));
  nor042aa1n03x5               g161(.a(new_n255), .b(new_n256), .o1(\s[22] ));
  nano23aa1n06x5               g162(.a(new_n245), .b(new_n251), .c(new_n252), .d(new_n248), .out0(new_n258));
  norb02aa1d21x5               g163(.a(new_n258), .b(new_n239), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n252), .b(new_n251), .c(new_n245), .out0(new_n261));
  aoi022aa1n03x5               g166(.a(new_n241), .b(new_n258), .c(new_n252), .d(new_n261), .o1(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n260), .c(new_n208), .d(new_n201), .o1(new_n263));
  xorb03aa1n02x5               g168(.a(new_n263), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  inv000aa1d42x5               g169(.a(\a[23] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(\b[22] ), .o1(new_n266));
  nand02aa1n12x5               g171(.a(new_n266), .b(new_n265), .o1(new_n267));
  inv040aa1n03x5               g172(.a(new_n262), .o1(new_n268));
  nanp02aa1n04x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  nanp02aa1n12x5               g174(.a(new_n267), .b(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n268), .c(new_n202), .d(new_n259), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\a[24] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(\b[23] ), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n274), .b(new_n273), .o1(new_n275));
  nand42aa1n02x5               g180(.a(\b[23] ), .b(\a[24] ), .o1(new_n276));
  nand02aa1d06x5               g181(.a(new_n275), .b(new_n276), .o1(new_n277));
  aoi012aa1n03x5               g182(.a(new_n277), .b(new_n272), .c(new_n267), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n267), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n277), .o1(new_n280));
  aoi112aa1n03x4               g185(.a(new_n279), .b(new_n280), .c(new_n263), .d(new_n269), .o1(new_n281));
  norp02aa1n03x5               g186(.a(new_n278), .b(new_n281), .o1(\s[24] ));
  norp02aa1n02x5               g187(.a(\b[23] ), .b(\a[24] ), .o1(new_n283));
  nanb03aa1n02x5               g188(.a(new_n283), .b(new_n276), .c(new_n269), .out0(new_n284));
  nona22aa1n12x5               g189(.a(new_n258), .b(new_n284), .c(new_n279), .out0(new_n285));
  nor002aa1n02x5               g190(.a(new_n239), .b(new_n285), .o1(new_n286));
  inv000aa1n02x5               g191(.a(new_n286), .o1(new_n287));
  nona22aa1n09x5               g192(.a(new_n227), .b(new_n219), .c(new_n233), .out0(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .o1(new_n289));
  oai012aa1n02x5               g194(.a(new_n252), .b(new_n251), .c(new_n245), .o1(new_n290));
  norp03aa1n06x5               g195(.a(new_n290), .b(new_n270), .c(new_n277), .o1(new_n291));
  nor042aa1n06x5               g196(.a(new_n291), .b(new_n289), .o1(new_n292));
  aoai13aa1n12x5               g197(.a(new_n292), .b(new_n285), .c(new_n288), .d(new_n240), .o1(new_n293));
  inv030aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n287), .c(new_n208), .d(new_n201), .o1(new_n295));
  xorb03aa1n02x5               g200(.a(new_n295), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g201(.a(\b[24] ), .b(\a[25] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  tech160nm_fixorc02aa1n03p5x5 g203(.a(\a[25] ), .b(\b[24] ), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n293), .c(new_n202), .d(new_n286), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[26] ), .b(\b[25] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  aoi012aa1n03x5               g207(.a(new_n302), .b(new_n300), .c(new_n298), .o1(new_n303));
  aoi112aa1n02x7               g208(.a(new_n297), .b(new_n301), .c(new_n295), .d(new_n299), .o1(new_n304));
  nor002aa1n02x5               g209(.a(new_n303), .b(new_n304), .o1(\s[26] ));
  nanb03aa1n02x5               g210(.a(new_n145), .b(new_n156), .c(new_n148), .out0(new_n306));
  oab012aa1n02x4               g211(.a(new_n198), .b(new_n199), .c(new_n178), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n206), .c(new_n306), .d(new_n165), .o1(new_n308));
  and002aa1n06x5               g213(.a(new_n301), .b(new_n299), .o(new_n309));
  norb03aa1d15x5               g214(.a(new_n309), .b(new_n239), .c(new_n285), .out0(new_n310));
  aoai13aa1n06x5               g215(.a(new_n310), .b(new_n308), .c(new_n143), .d(new_n207), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[26] ), .b(\b[25] ), .c(new_n298), .carry(new_n312));
  aobi12aa1n12x5               g217(.a(new_n312), .b(new_n293), .c(new_n309), .out0(new_n313));
  xorc02aa1n12x5               g218(.a(\a[27] ), .b(\b[26] ), .out0(new_n314));
  xnbna2aa1n03x5               g219(.a(new_n314), .b(new_n311), .c(new_n313), .out0(\s[27] ));
  norp02aa1n02x5               g220(.a(\b[26] ), .b(\a[27] ), .o1(new_n316));
  inv040aa1n03x5               g221(.a(new_n316), .o1(new_n317));
  nano23aa1n02x4               g222(.a(new_n261), .b(new_n284), .c(new_n267), .d(new_n248), .out0(new_n318));
  nand02aa1d04x5               g223(.a(new_n241), .b(new_n318), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n309), .o1(new_n320));
  aoai13aa1n12x5               g225(.a(new_n312), .b(new_n320), .c(new_n319), .d(new_n292), .o1(new_n321));
  aoai13aa1n06x5               g226(.a(new_n314), .b(new_n321), .c(new_n202), .d(new_n310), .o1(new_n322));
  xnrc02aa1n12x5               g227(.a(\b[27] ), .b(\a[28] ), .out0(new_n323));
  tech160nm_fiaoi012aa1n05x5   g228(.a(new_n323), .b(new_n322), .c(new_n317), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n314), .o1(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n325), .b(new_n311), .c(new_n313), .o1(new_n326));
  nano22aa1n03x5               g231(.a(new_n326), .b(new_n317), .c(new_n323), .out0(new_n327));
  norp02aa1n03x5               g232(.a(new_n324), .b(new_n327), .o1(\s[28] ));
  norb02aa1d21x5               g233(.a(new_n314), .b(new_n323), .out0(new_n329));
  aoai13aa1n06x5               g234(.a(new_n329), .b(new_n321), .c(new_n202), .d(new_n310), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[28] ), .b(\b[27] ), .c(new_n317), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[28] ), .b(\a[29] ), .out0(new_n332));
  aoi012aa1n03x5               g237(.a(new_n332), .b(new_n330), .c(new_n331), .o1(new_n333));
  inv000aa1d42x5               g238(.a(new_n329), .o1(new_n334));
  tech160nm_fiaoi012aa1n02p5x5 g239(.a(new_n334), .b(new_n311), .c(new_n313), .o1(new_n335));
  nano22aa1n03x5               g240(.a(new_n335), .b(new_n331), .c(new_n332), .out0(new_n336));
  norp02aa1n03x5               g241(.a(new_n333), .b(new_n336), .o1(\s[29] ));
  xorb03aa1n02x5               g242(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g243(.a(new_n314), .b(new_n332), .c(new_n323), .out0(new_n339));
  aoai13aa1n06x5               g244(.a(new_n339), .b(new_n321), .c(new_n202), .d(new_n310), .o1(new_n340));
  oao003aa1n02x5               g245(.a(\a[29] ), .b(\b[28] ), .c(new_n331), .carry(new_n341));
  xnrc02aa1n02x5               g246(.a(\b[29] ), .b(\a[30] ), .out0(new_n342));
  tech160nm_fiaoi012aa1n05x5   g247(.a(new_n342), .b(new_n340), .c(new_n341), .o1(new_n343));
  inv000aa1d42x5               g248(.a(new_n339), .o1(new_n344));
  tech160nm_fiaoi012aa1n02p5x5 g249(.a(new_n344), .b(new_n311), .c(new_n313), .o1(new_n345));
  nano22aa1n02x4               g250(.a(new_n345), .b(new_n341), .c(new_n342), .out0(new_n346));
  norp02aa1n03x5               g251(.a(new_n343), .b(new_n346), .o1(\s[30] ));
  xnrc02aa1n02x5               g252(.a(\b[30] ), .b(\a[31] ), .out0(new_n348));
  nona32aa1n02x4               g253(.a(new_n314), .b(new_n342), .c(new_n332), .d(new_n323), .out0(new_n349));
  inv000aa1n02x5               g254(.a(new_n349), .o1(new_n350));
  aoai13aa1n06x5               g255(.a(new_n350), .b(new_n321), .c(new_n202), .d(new_n310), .o1(new_n351));
  oao003aa1n02x5               g256(.a(\a[30] ), .b(\b[29] ), .c(new_n341), .carry(new_n352));
  aoi012aa1n03x5               g257(.a(new_n348), .b(new_n351), .c(new_n352), .o1(new_n353));
  tech160nm_fiaoi012aa1n02p5x5 g258(.a(new_n349), .b(new_n311), .c(new_n313), .o1(new_n354));
  nano22aa1n03x5               g259(.a(new_n354), .b(new_n348), .c(new_n352), .out0(new_n355));
  norp02aa1n03x5               g260(.a(new_n353), .b(new_n355), .o1(\s[31] ));
  xnbna2aa1n03x5               g261(.a(new_n134), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  norb02aa1n02x5               g262(.a(new_n105), .b(new_n106), .out0(new_n358));
  aoi012aa1n02x5               g263(.a(new_n99), .b(new_n135), .c(new_n134), .o1(new_n359));
  oai012aa1n02x5               g264(.a(new_n108), .b(new_n359), .c(new_n358), .o1(\s[4] ));
  norb02aa1n02x5               g265(.a(new_n113), .b(new_n124), .out0(new_n361));
  xobna2aa1n03x5               g266(.a(new_n361), .b(new_n108), .c(new_n105), .out0(\s[5] ));
  norb02aa1n02x5               g267(.a(new_n109), .b(new_n123), .out0(new_n363));
  nanp03aa1n02x5               g268(.a(new_n108), .b(new_n105), .c(new_n361), .o1(new_n364));
  xnbna2aa1n03x5               g269(.a(new_n363), .b(new_n364), .c(new_n115), .out0(\s[6] ));
  nanp03aa1n02x5               g270(.a(new_n364), .b(new_n115), .c(new_n363), .o1(new_n366));
  nanp02aa1n02x5               g271(.a(new_n366), .b(new_n109), .o1(new_n367));
  xnrb03aa1n02x5               g272(.a(new_n367), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g273(.a(\a[7] ), .b(\b[6] ), .c(new_n367), .o1(new_n369));
  xorb03aa1n02x5               g274(.a(new_n369), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g275(.a(new_n191), .b(new_n127), .c(new_n98), .out0(\s[9] ));
endmodule

