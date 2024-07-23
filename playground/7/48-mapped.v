// Benchmark "adder" written by ABC on Wed Jul 17 15:58:06 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n351, new_n353,
    new_n355, new_n357, new_n358, new_n360, new_n362, new_n363, new_n364;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nand22aa1n03x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  aoi012aa1n06x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1n10x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  aoi012aa1n06x5               g015(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n111));
  oaih12aa1n06x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  tech160nm_fixnrc02aa1n04x5   g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xnrc02aa1n12x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  nor022aa1n08x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand22aa1n12x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand42aa1n06x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norp02aa1n06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n117), .b(new_n116), .c(new_n118), .d(new_n115), .out0(new_n119));
  norp03aa1d12x5               g024(.a(new_n119), .b(new_n114), .c(new_n113), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[6] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[5] ), .o1(new_n122));
  nor042aa1n09x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .o1(new_n124));
  aoi012aa1n02x7               g029(.a(new_n115), .b(new_n118), .c(new_n116), .o1(new_n125));
  oai012aa1n06x5               g030(.a(new_n125), .b(new_n119), .c(new_n124), .o1(new_n126));
  nand42aa1n08x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n100), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  nor042aa1n04x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  inv000aa1d42x5               g038(.a(\a[10] ), .o1(new_n134));
  nano23aa1n02x4               g039(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n135));
  nanb02aa1n03x5               g040(.a(new_n105), .b(new_n135), .out0(new_n136));
  nano23aa1n03x5               g041(.a(new_n118), .b(new_n115), .c(new_n116), .d(new_n117), .out0(new_n137));
  nona22aa1n03x5               g042(.a(new_n137), .b(new_n114), .c(new_n113), .out0(new_n138));
  inv030aa1n02x5               g043(.a(new_n123), .o1(new_n139));
  oaoi03aa1n02x5               g044(.a(\a[6] ), .b(\b[5] ), .c(new_n139), .o1(new_n140));
  aobi12aa1n02x7               g045(.a(new_n125), .b(new_n137), .c(new_n140), .out0(new_n141));
  aoai13aa1n04x5               g046(.a(new_n141), .b(new_n138), .c(new_n136), .d(new_n111), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n98), .b(new_n100), .c(new_n142), .d(new_n127), .o1(new_n143));
  oaib12aa1n06x5               g048(.a(new_n143), .b(\b[9] ), .c(new_n134), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n129), .b(new_n101), .o1(new_n145));
  oai012aa1n02x5               g050(.a(new_n98), .b(new_n145), .c(new_n97), .o1(new_n146));
  mtn022aa1n02x5               g051(.a(new_n144), .b(new_n146), .sa(new_n133), .o1(\s[11] ));
  nor042aa1n09x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand02aa1d10x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n09x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n03x5               g056(.a(new_n151), .b(new_n131), .c(new_n144), .d(new_n132), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n133), .b(new_n97), .c(new_n145), .d(new_n98), .o1(new_n153));
  nona22aa1n02x4               g058(.a(new_n153), .b(new_n151), .c(new_n131), .out0(new_n154));
  nanp02aa1n02x5               g059(.a(new_n152), .b(new_n154), .o1(\s[12] ));
  nano32aa1n03x7               g060(.a(new_n151), .b(new_n133), .c(new_n128), .d(new_n99), .out0(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n157));
  nano22aa1n03x7               g062(.a(new_n148), .b(new_n132), .c(new_n149), .out0(new_n158));
  oai122aa1n12x5               g063(.a(new_n98), .b(new_n97), .c(new_n100), .d(\b[10] ), .e(\a[11] ), .o1(new_n159));
  aoi012aa1d18x5               g064(.a(new_n148), .b(new_n131), .c(new_n149), .o1(new_n160));
  oaib12aa1n18x5               g065(.a(new_n160), .b(new_n159), .c(new_n158), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(new_n157), .b(new_n162), .o1(new_n163));
  nor022aa1n16x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nanb02aa1n02x5               g071(.a(new_n159), .b(new_n158), .out0(new_n167));
  nano22aa1n02x4               g072(.a(new_n166), .b(new_n167), .c(new_n160), .out0(new_n168));
  aoi022aa1n02x5               g073(.a(new_n163), .b(new_n166), .c(new_n157), .d(new_n168), .o1(\s[13] ));
  aoi012aa1n02x5               g074(.a(new_n164), .b(new_n163), .c(new_n165), .o1(new_n170));
  xnrb03aa1n02x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n20x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nona23aa1n02x5               g078(.a(new_n173), .b(new_n165), .c(new_n164), .d(new_n172), .out0(new_n174));
  oa0012aa1n02x5               g079(.a(new_n173), .b(new_n172), .c(new_n164), .o(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n176), .b(new_n174), .c(new_n157), .d(new_n162), .o1(new_n177));
  nor042aa1n04x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand42aa1d28x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  nano23aa1n09x5               g085(.a(new_n164), .b(new_n172), .c(new_n173), .d(new_n165), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(new_n180), .b(new_n175), .c(new_n163), .d(new_n181), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n182), .b(new_n177), .c(new_n180), .o1(\s[15] ));
  norp02aa1n06x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nand02aa1n16x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n184), .b(new_n185), .out0(new_n186));
  aoai13aa1n02x5               g091(.a(new_n186), .b(new_n178), .c(new_n177), .d(new_n179), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n178), .b(new_n186), .c(new_n177), .d(new_n179), .o1(new_n188));
  nanb02aa1n02x5               g093(.a(new_n188), .b(new_n187), .out0(\s[16] ));
  nano23aa1n02x4               g094(.a(new_n97), .b(new_n100), .c(new_n127), .d(new_n98), .out0(new_n190));
  nano23aa1n06x5               g095(.a(new_n131), .b(new_n148), .c(new_n149), .d(new_n132), .out0(new_n191));
  nano23aa1n06x5               g096(.a(new_n178), .b(new_n184), .c(new_n185), .d(new_n179), .out0(new_n192));
  nand42aa1n02x5               g097(.a(new_n192), .b(new_n181), .o1(new_n193));
  nano22aa1n03x7               g098(.a(new_n193), .b(new_n190), .c(new_n191), .out0(new_n194));
  aoai13aa1n12x5               g099(.a(new_n194), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n195));
  norb02aa1n03x5               g100(.a(new_n192), .b(new_n174), .out0(new_n196));
  nanb03aa1n06x5               g101(.a(new_n184), .b(new_n185), .c(new_n179), .out0(new_n197));
  oai122aa1n06x5               g102(.a(new_n173), .b(new_n172), .c(new_n164), .d(\b[14] ), .e(\a[15] ), .o1(new_n198));
  aoi012aa1n02x7               g103(.a(new_n184), .b(new_n178), .c(new_n185), .o1(new_n199));
  oai012aa1n06x5               g104(.a(new_n199), .b(new_n198), .c(new_n197), .o1(new_n200));
  aoi012aa1d24x5               g105(.a(new_n200), .b(new_n161), .c(new_n196), .o1(new_n201));
  nand02aa1d08x5               g106(.a(new_n195), .b(new_n201), .o1(new_n202));
  nor002aa1n06x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  nand42aa1n08x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  oaib12aa1n02x5               g110(.a(new_n199), .b(new_n203), .c(new_n204), .out0(new_n206));
  oabi12aa1n02x5               g111(.a(new_n206), .b(new_n197), .c(new_n198), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n207), .b(new_n161), .c(new_n196), .o1(new_n208));
  aoi022aa1n02x5               g113(.a(new_n202), .b(new_n205), .c(new_n195), .d(new_n208), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g114(.a(new_n203), .b(new_n202), .c(new_n205), .o1(new_n210));
  xnrb03aa1n03x5               g115(.a(new_n210), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n04x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nand42aa1n08x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nano23aa1n06x5               g118(.a(new_n203), .b(new_n212), .c(new_n213), .d(new_n204), .out0(new_n214));
  oa0012aa1n02x5               g119(.a(new_n213), .b(new_n212), .c(new_n203), .o(new_n215));
  nor042aa1n02x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nand02aa1n03x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  norb02aa1n06x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n215), .c(new_n202), .d(new_n214), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(new_n218), .b(new_n215), .c(new_n202), .d(new_n214), .o1(new_n220));
  norb02aa1n02x7               g125(.a(new_n219), .b(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g127(.a(\b[19] ), .o1(new_n223));
  nanb02aa1n02x5               g128(.a(\a[20] ), .b(new_n223), .out0(new_n224));
  nand42aa1n04x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n224), .b(new_n225), .o1(new_n226));
  inv040aa1d32x5               g131(.a(\a[19] ), .o1(new_n227));
  inv020aa1d32x5               g132(.a(\b[18] ), .o1(new_n228));
  nand42aa1n02x5               g133(.a(new_n228), .b(new_n227), .o1(new_n229));
  tech160nm_finand02aa1n03p5x5 g134(.a(new_n219), .b(new_n229), .o1(new_n230));
  nanp02aa1n03x5               g135(.a(new_n230), .b(new_n226), .o1(new_n231));
  nona22aa1n02x5               g136(.a(new_n219), .b(new_n226), .c(new_n216), .out0(new_n232));
  nanp02aa1n03x5               g137(.a(new_n231), .b(new_n232), .o1(\s[20] ));
  nanb03aa1n06x5               g138(.a(new_n226), .b(new_n214), .c(new_n218), .out0(new_n234));
  inv040aa1n06x5               g139(.a(new_n234), .o1(new_n235));
  nor042aa1n04x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  nanb03aa1n03x5               g141(.a(new_n236), .b(new_n225), .c(new_n217), .out0(new_n237));
  oai022aa1n02x5               g142(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n238));
  nanp03aa1n02x5               g143(.a(new_n238), .b(new_n229), .c(new_n213), .o1(new_n239));
  aoi012aa1n09x5               g144(.a(new_n236), .b(new_n216), .c(new_n225), .o1(new_n240));
  oai012aa1n06x5               g145(.a(new_n240), .b(new_n239), .c(new_n237), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[20] ), .b(\a[21] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n241), .c(new_n202), .d(new_n235), .o1(new_n244));
  oai112aa1n02x5               g149(.a(new_n240), .b(new_n242), .c(new_n239), .d(new_n237), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n202), .c(new_n235), .o1(new_n246));
  norb02aa1n02x7               g151(.a(new_n244), .b(new_n246), .out0(\s[21] ));
  xnrc02aa1n06x5               g152(.a(\b[21] ), .b(\a[22] ), .out0(new_n248));
  tech160nm_fioai012aa1n03p5x5 g153(.a(new_n244), .b(\b[20] ), .c(\a[21] ), .o1(new_n249));
  nand02aa1n02x5               g154(.a(new_n249), .b(new_n248), .o1(new_n250));
  nor002aa1n02x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  nona22aa1n02x5               g156(.a(new_n244), .b(new_n248), .c(new_n251), .out0(new_n252));
  nanp02aa1n03x5               g157(.a(new_n250), .b(new_n252), .o1(\s[22] ));
  nona32aa1n02x4               g158(.a(new_n202), .b(new_n248), .c(new_n242), .d(new_n234), .out0(new_n254));
  nor042aa1n06x5               g159(.a(new_n248), .b(new_n242), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n234), .b(new_n255), .out0(new_n256));
  inv000aa1d42x5               g161(.a(\a[22] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(\b[21] ), .o1(new_n258));
  oao003aa1n06x5               g163(.a(new_n257), .b(new_n258), .c(new_n251), .carry(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n241), .c(new_n255), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n260), .b(new_n256), .c(new_n195), .d(new_n201), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n262), .b(new_n259), .c(new_n241), .d(new_n255), .o1(new_n263));
  aoi022aa1n02x5               g168(.a(new_n263), .b(new_n254), .c(new_n261), .d(new_n262), .o1(\s[23] ));
  norp02aa1n02x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  xnrc02aa1n12x5               g170(.a(\b[23] ), .b(\a[24] ), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n265), .c(new_n261), .d(new_n262), .o1(new_n267));
  aoi112aa1n03x4               g172(.a(new_n265), .b(new_n266), .c(new_n261), .d(new_n262), .o1(new_n268));
  nanb02aa1n03x5               g173(.a(new_n268), .b(new_n267), .out0(\s[24] ));
  inv040aa1n03x5               g174(.a(new_n200), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n193), .c(new_n167), .d(new_n160), .o1(new_n271));
  inv000aa1n02x5               g176(.a(new_n255), .o1(new_n272));
  nanb02aa1n18x5               g177(.a(new_n266), .b(new_n262), .out0(new_n273));
  nor043aa1n03x5               g178(.a(new_n234), .b(new_n272), .c(new_n273), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n271), .c(new_n142), .d(new_n194), .o1(new_n275));
  nano22aa1n02x4               g180(.a(new_n236), .b(new_n217), .c(new_n225), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .o1(new_n277));
  oab012aa1n06x5               g182(.a(new_n277), .b(new_n203), .c(new_n212), .out0(new_n278));
  inv030aa1n02x5               g183(.a(new_n240), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n255), .b(new_n279), .c(new_n278), .d(new_n276), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n259), .o1(new_n281));
  aoi112aa1n02x5               g186(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n282));
  oab012aa1n02x4               g187(.a(new_n282), .b(\a[24] ), .c(\b[23] ), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n273), .c(new_n280), .d(new_n281), .o1(new_n284));
  nanb02aa1n02x5               g189(.a(new_n284), .b(new_n275), .out0(new_n285));
  xorc02aa1n12x5               g190(.a(\a[25] ), .b(\b[24] ), .out0(new_n286));
  inv020aa1n02x5               g191(.a(new_n273), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n287), .b(new_n259), .c(new_n241), .d(new_n255), .o1(new_n288));
  nano22aa1n02x4               g193(.a(new_n286), .b(new_n288), .c(new_n283), .out0(new_n289));
  aoi022aa1n02x5               g194(.a(new_n285), .b(new_n286), .c(new_n275), .d(new_n289), .o1(\s[25] ));
  norp02aa1n02x5               g195(.a(\b[24] ), .b(\a[25] ), .o1(new_n291));
  tech160nm_fixnrc02aa1n04x5   g196(.a(\b[25] ), .b(\a[26] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n285), .d(new_n286), .o1(new_n293));
  aoai13aa1n02x5               g198(.a(new_n286), .b(new_n284), .c(new_n202), .d(new_n274), .o1(new_n294));
  nona22aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n291), .out0(new_n295));
  nanp02aa1n02x5               g200(.a(new_n293), .b(new_n295), .o1(\s[26] ));
  norb02aa1n12x5               g201(.a(new_n286), .b(new_n292), .out0(new_n297));
  nano23aa1n06x5               g202(.a(new_n234), .b(new_n273), .c(new_n297), .d(new_n255), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n271), .c(new_n142), .d(new_n194), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n300));
  oab012aa1n02x4               g205(.a(new_n300), .b(\a[26] ), .c(\b[25] ), .out0(new_n301));
  aobi12aa1n06x5               g206(.a(new_n301), .b(new_n284), .c(new_n297), .out0(new_n302));
  xorc02aa1n12x5               g207(.a(\a[27] ), .b(\b[26] ), .out0(new_n303));
  xnbna2aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n299), .out0(\s[27] ));
  nand42aa1n02x5               g209(.a(new_n302), .b(new_n299), .o1(new_n305));
  norp02aa1n02x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  norp02aa1n02x5               g211(.a(\b[27] ), .b(\a[28] ), .o1(new_n307));
  nanp02aa1n02x5               g212(.a(\b[27] ), .b(\a[28] ), .o1(new_n308));
  norb02aa1n02x5               g213(.a(new_n308), .b(new_n307), .out0(new_n309));
  inv040aa1n03x5               g214(.a(new_n309), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n306), .c(new_n305), .d(new_n303), .o1(new_n311));
  nona23aa1n03x5               g216(.a(new_n235), .b(new_n297), .c(new_n273), .d(new_n272), .out0(new_n312));
  aoi012aa1n06x5               g217(.a(new_n312), .b(new_n195), .c(new_n201), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n297), .o1(new_n314));
  aoai13aa1n04x5               g219(.a(new_n301), .b(new_n314), .c(new_n288), .d(new_n283), .o1(new_n315));
  oaih12aa1n02x5               g220(.a(new_n303), .b(new_n315), .c(new_n313), .o1(new_n316));
  nona22aa1n02x5               g221(.a(new_n316), .b(new_n310), .c(new_n306), .out0(new_n317));
  nanp02aa1n03x5               g222(.a(new_n311), .b(new_n317), .o1(\s[28] ));
  xnrc02aa1n02x5               g223(.a(\b[28] ), .b(\a[29] ), .out0(new_n319));
  norb02aa1d27x5               g224(.a(new_n303), .b(new_n310), .out0(new_n320));
  oaih12aa1n02x5               g225(.a(new_n320), .b(new_n315), .c(new_n313), .o1(new_n321));
  oai012aa1n02x5               g226(.a(new_n308), .b(new_n307), .c(new_n306), .o1(new_n322));
  tech160nm_fiaoi012aa1n02p5x5 g227(.a(new_n319), .b(new_n321), .c(new_n322), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n320), .o1(new_n324));
  aoi012aa1n06x5               g229(.a(new_n324), .b(new_n302), .c(new_n299), .o1(new_n325));
  nano22aa1n03x5               g230(.a(new_n325), .b(new_n319), .c(new_n322), .out0(new_n326));
  norp02aa1n03x5               g231(.a(new_n323), .b(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g233(.a(new_n319), .b(new_n303), .c(new_n309), .out0(new_n329));
  oabi12aa1n02x7               g234(.a(new_n329), .b(new_n315), .c(new_n313), .out0(new_n330));
  inv000aa1d42x5               g235(.a(\b[28] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\a[29] ), .o1(new_n332));
  oaib12aa1n02x5               g237(.a(new_n322), .b(\b[28] ), .c(new_n332), .out0(new_n333));
  oaib12aa1n02x5               g238(.a(new_n333), .b(new_n331), .c(\a[29] ), .out0(new_n334));
  aoai13aa1n02x7               g239(.a(new_n334), .b(new_n329), .c(new_n302), .d(new_n299), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .out0(new_n336));
  oaoi13aa1n02x5               g241(.a(new_n336), .b(new_n333), .c(new_n332), .d(new_n331), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n335), .b(new_n336), .c(new_n330), .d(new_n337), .o1(\s[30] ));
  nano32aa1n02x4               g243(.a(new_n319), .b(new_n336), .c(new_n303), .d(new_n309), .out0(new_n339));
  oaih12aa1n02x5               g244(.a(new_n339), .b(new_n315), .c(new_n313), .o1(new_n340));
  aoi022aa1n02x5               g245(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n341));
  norb02aa1n02x5               g246(.a(\a[31] ), .b(\b[30] ), .out0(new_n342));
  obai22aa1n02x7               g247(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n343));
  aoi112aa1n02x5               g248(.a(new_n343), .b(new_n342), .c(new_n333), .d(new_n341), .o1(new_n344));
  norp02aa1n02x5               g249(.a(\b[29] ), .b(\a[30] ), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n333), .c(new_n341), .o1(new_n346));
  nand42aa1n02x5               g251(.a(new_n340), .b(new_n346), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[31] ), .b(\b[30] ), .out0(new_n348));
  aoi022aa1n02x7               g253(.a(new_n347), .b(new_n348), .c(new_n340), .d(new_n344), .o1(\s[31] ));
  xnrb03aa1n02x5               g254(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g255(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n351));
  xorb03aa1n02x5               g256(.a(new_n351), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  inv000aa1d42x5               g257(.a(new_n114), .o1(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n136), .c(new_n111), .out0(\s[5] ));
  nanp02aa1n03x5               g259(.a(new_n112), .b(new_n353), .o1(new_n355));
  xobna2aa1n03x5               g260(.a(new_n113), .b(new_n355), .c(new_n139), .out0(\s[6] ));
  aoi012aa1n02x5               g261(.a(new_n113), .b(new_n355), .c(new_n139), .o1(new_n357));
  tech160nm_fiaoi012aa1n05x5   g262(.a(new_n357), .b(new_n121), .c(new_n122), .o1(new_n358));
  xnrb03aa1n02x5               g263(.a(new_n358), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g264(.a(\a[7] ), .b(\b[6] ), .c(new_n358), .o1(new_n360));
  xorb03aa1n02x5               g265(.a(new_n360), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  nanp02aa1n02x5               g266(.a(new_n112), .b(new_n120), .o1(new_n362));
  oaib12aa1n02x5               g267(.a(new_n125), .b(new_n100), .c(new_n127), .out0(new_n363));
  aoi012aa1n02x5               g268(.a(new_n363), .b(new_n137), .c(new_n140), .o1(new_n364));
  aoi022aa1n02x5               g269(.a(new_n142), .b(new_n128), .c(new_n362), .d(new_n364), .o1(\s[9] ));
endmodule


