// Benchmark "adder" written by ABC on Thu Jul 18 00:00:11 2024

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
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n326,
    new_n327, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv030aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv030aa1d28x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[2] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[1] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oaoi03aa1n09x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor022aa1n08x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1d04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  ao0012aa1n03x5               g013(.a(new_n104), .b(new_n106), .c(new_n105), .o(new_n109));
  oabi12aa1n06x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .out0(new_n110));
  xorc02aa1n02x5               g015(.a(\a[6] ), .b(\b[5] ), .out0(new_n111));
  nor042aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norb02aa1n06x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  orn002aa1n24x5               g019(.a(\a[7] ), .b(\b[6] ), .o(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand23aa1n03x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  xorc02aa1n03x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nano22aa1n03x7               g023(.a(new_n117), .b(new_n111), .c(new_n118), .out0(new_n119));
  nanp02aa1n03x5               g024(.a(new_n119), .b(new_n110), .o1(new_n120));
  aoi112aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n121));
  nand42aa1n04x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n112), .b(new_n113), .out0(new_n123));
  nanp02aa1n06x5               g028(.a(new_n115), .b(new_n116), .o1(new_n124));
  oai022aa1n02x5               g029(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n125));
  nano23aa1n03x7               g030(.a(new_n124), .b(new_n123), .c(new_n125), .d(new_n122), .out0(new_n126));
  nor043aa1n03x5               g031(.a(new_n126), .b(new_n121), .c(new_n112), .o1(new_n127));
  tech160nm_fixnrc02aa1n04x5   g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n99), .b(new_n128), .c(new_n120), .d(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanb02aa1d36x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  oao003aa1n02x5               g038(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n134));
  nano23aa1n02x4               g039(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n135));
  aoi012aa1n02x5               g040(.a(new_n109), .b(new_n135), .c(new_n134), .o1(new_n136));
  nona23aa1n02x4               g041(.a(new_n111), .b(new_n118), .c(new_n124), .d(new_n123), .out0(new_n137));
  oai012aa1n02x5               g042(.a(new_n127), .b(new_n136), .c(new_n137), .o1(new_n138));
  oaoi03aa1n02x5               g043(.a(new_n97), .b(new_n98), .c(new_n138), .o1(new_n139));
  nor042aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand02aa1n04x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n03x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  aoai13aa1n12x5               g048(.a(new_n132), .b(new_n131), .c(new_n97), .d(new_n98), .o1(new_n144));
  oaoi13aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(new_n139), .d(new_n133), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n133), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n144), .o1(new_n147));
  aoi112aa1n02x5               g052(.a(new_n142), .b(new_n147), .c(new_n129), .d(new_n146), .o1(new_n148));
  norp02aa1n02x5               g053(.a(new_n145), .b(new_n148), .o1(\s[11] ));
  aoai13aa1n06x5               g054(.a(new_n142), .b(new_n147), .c(new_n129), .d(new_n146), .o1(new_n150));
  nor042aa1n12x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nanp02aa1n04x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  oai112aa1n02x5               g058(.a(new_n150), .b(new_n153), .c(\b[10] ), .d(\a[11] ), .o1(new_n154));
  oaoi13aa1n04x5               g059(.a(new_n153), .b(new_n150), .c(\a[11] ), .d(\b[10] ), .o1(new_n155));
  norb02aa1n03x4               g060(.a(new_n154), .b(new_n155), .out0(\s[12] ));
  nano23aa1n03x7               g061(.a(new_n140), .b(new_n151), .c(new_n152), .d(new_n141), .out0(new_n157));
  nona22aa1n03x5               g062(.a(new_n157), .b(new_n128), .c(new_n133), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n151), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n152), .b(new_n141), .c(new_n140), .d(new_n151), .out0(new_n160));
  nanp02aa1n02x5               g065(.a(new_n140), .b(new_n152), .o1(new_n161));
  oai112aa1n06x5               g066(.a(new_n161), .b(new_n159), .c(new_n160), .d(new_n144), .o1(new_n162));
  inv040aa1n03x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n158), .c(new_n120), .d(new_n127), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nanp02aa1n04x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n166), .b(new_n164), .c(new_n167), .o1(new_n168));
  xnrb03aa1n02x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand42aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n06x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  inv000aa1n02x5               g077(.a(new_n172), .o1(new_n173));
  nor042aa1d18x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  tech160nm_finand02aa1n05x5   g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nano23aa1n06x5               g080(.a(new_n166), .b(new_n174), .c(new_n175), .d(new_n167), .out0(new_n176));
  oai012aa1n04x7               g081(.a(new_n175), .b(new_n174), .c(new_n166), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n173), .b(new_n178), .c(new_n164), .d(new_n176), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n173), .b(new_n178), .c(new_n164), .d(new_n176), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(\s[15] ));
  norp02aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  norb02aa1n03x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  nona22aa1n02x4               g089(.a(new_n179), .b(new_n184), .c(new_n170), .out0(new_n185));
  nanb02aa1n02x5               g090(.a(new_n182), .b(new_n183), .out0(new_n186));
  oaoi13aa1n06x5               g091(.a(new_n186), .b(new_n179), .c(\a[15] ), .d(\b[14] ), .o1(new_n187));
  norb02aa1n03x4               g092(.a(new_n185), .b(new_n187), .out0(\s[16] ));
  inv040aa1n02x5               g093(.a(new_n122), .o1(new_n189));
  xorc02aa1n02x5               g094(.a(\a[7] ), .b(\b[6] ), .out0(new_n190));
  norp02aa1n04x5               g095(.a(\b[5] ), .b(\a[6] ), .o1(new_n191));
  nor002aa1n03x5               g096(.a(\b[4] ), .b(\a[5] ), .o1(new_n192));
  nor022aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nona23aa1n02x4               g098(.a(new_n190), .b(new_n114), .c(new_n193), .d(new_n189), .out0(new_n194));
  nona22aa1n02x4               g099(.a(new_n194), .b(new_n121), .c(new_n112), .out0(new_n195));
  nano32aa1n03x7               g100(.a(new_n158), .b(new_n184), .c(new_n173), .d(new_n176), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n195), .c(new_n110), .d(new_n119), .o1(new_n197));
  nona23aa1n09x5               g102(.a(new_n183), .b(new_n171), .c(new_n170), .d(new_n182), .out0(new_n198));
  norb02aa1n02x7               g103(.a(new_n176), .b(new_n198), .out0(new_n199));
  aoi112aa1n02x5               g104(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n200));
  oai022aa1n03x5               g105(.a(new_n198), .b(new_n177), .c(\b[15] ), .d(\a[16] ), .o1(new_n201));
  aoi112aa1n09x5               g106(.a(new_n201), .b(new_n200), .c(new_n162), .d(new_n199), .o1(new_n202));
  nand02aa1d10x5               g107(.a(new_n202), .b(new_n197), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g109(.a(\a[18] ), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\a[17] ), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\b[16] ), .o1(new_n207));
  oaoi03aa1n02x5               g112(.a(new_n206), .b(new_n207), .c(new_n203), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[17] ), .c(new_n205), .out0(\s[18] ));
  xroi22aa1d06x4               g114(.a(new_n206), .b(\b[16] ), .c(new_n205), .d(\b[17] ), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n207), .b(new_n206), .o1(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n212));
  nor022aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand42aa1n08x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n212), .c(new_n203), .d(new_n210), .o1(new_n216));
  aoi112aa1n02x5               g121(.a(new_n215), .b(new_n212), .c(new_n203), .d(new_n210), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand42aa1n06x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  nona22aa1n03x5               g127(.a(new_n216), .b(new_n222), .c(new_n213), .out0(new_n223));
  orn002aa1n24x5               g128(.a(\a[19] ), .b(\b[18] ), .o(new_n224));
  aobi12aa1n02x7               g129(.a(new_n222), .b(new_n216), .c(new_n224), .out0(new_n225));
  norb02aa1n03x4               g130(.a(new_n223), .b(new_n225), .out0(\s[20] ));
  nano23aa1n03x5               g131(.a(new_n213), .b(new_n220), .c(new_n221), .d(new_n214), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n210), .b(new_n227), .o1(new_n228));
  oai022aa1n02x5               g133(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n229));
  oaib12aa1n09x5               g134(.a(new_n229), .b(new_n205), .c(\b[17] ), .out0(new_n230));
  nona23aa1n06x5               g135(.a(new_n221), .b(new_n214), .c(new_n213), .d(new_n220), .out0(new_n231));
  tech160nm_fioaoi03aa1n02p5x5 g136(.a(\a[20] ), .b(\b[19] ), .c(new_n224), .o1(new_n232));
  oabi12aa1n18x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n228), .c(new_n202), .d(new_n197), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[21] ), .b(\b[20] ), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  aoi112aa1n02x7               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n03x4               g146(.a(new_n241), .b(new_n240), .out0(\s[22] ));
  inv000aa1d42x5               g147(.a(\a[21] ), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\a[22] ), .o1(new_n244));
  xroi22aa1d04x5               g149(.a(new_n243), .b(\b[20] ), .c(new_n244), .d(\b[21] ), .out0(new_n245));
  nanp03aa1n02x5               g150(.a(new_n245), .b(new_n210), .c(new_n227), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\b[21] ), .o1(new_n247));
  oaoi03aa1n12x5               g152(.a(new_n244), .b(new_n247), .c(new_n237), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  aoi012aa1n02x5               g154(.a(new_n249), .b(new_n233), .c(new_n245), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n246), .c(new_n202), .d(new_n197), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  xorc02aa1n03x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n253), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n257));
  norb02aa1n03x4               g162(.a(new_n257), .b(new_n256), .out0(\s[24] ));
  and002aa1n06x5               g163(.a(new_n255), .b(new_n254), .o(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  nano32aa1n02x4               g165(.a(new_n260), .b(new_n245), .c(new_n210), .d(new_n227), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n245), .b(new_n232), .c(new_n227), .d(new_n212), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n263));
  oab012aa1n02x4               g168(.a(new_n263), .b(\a[24] ), .c(\b[23] ), .out0(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n260), .c(new_n262), .d(new_n248), .o1(new_n265));
  tech160nm_fixorc02aa1n05x5   g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n265), .c(new_n203), .d(new_n261), .o1(new_n267));
  aoi112aa1n02x5               g172(.a(new_n266), .b(new_n265), .c(new_n203), .d(new_n261), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n267), .b(new_n268), .out0(\s[25] ));
  nor042aa1n03x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  xorc02aa1n03x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  nona22aa1n03x5               g176(.a(new_n267), .b(new_n271), .c(new_n270), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n270), .o1(new_n273));
  aobi12aa1n02x7               g178(.a(new_n271), .b(new_n267), .c(new_n273), .out0(new_n274));
  norb02aa1n03x4               g179(.a(new_n272), .b(new_n274), .out0(\s[26] ));
  and002aa1n12x5               g180(.a(new_n271), .b(new_n266), .o(new_n276));
  nano22aa1n03x7               g181(.a(new_n246), .b(new_n259), .c(new_n276), .out0(new_n277));
  nand02aa1d10x5               g182(.a(new_n203), .b(new_n277), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .carry(new_n279));
  aobi12aa1n06x5               g184(.a(new_n279), .b(new_n265), .c(new_n276), .out0(new_n280));
  xorc02aa1n06x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  aobi12aa1n06x5               g189(.a(new_n281), .b(new_n278), .c(new_n280), .out0(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  nano22aa1n03x7               g191(.a(new_n285), .b(new_n284), .c(new_n286), .out0(new_n287));
  inv020aa1n03x5               g192(.a(new_n277), .o1(new_n288));
  aoi012aa1n06x5               g193(.a(new_n288), .b(new_n202), .c(new_n197), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n259), .b(new_n249), .c(new_n233), .d(new_n245), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n276), .o1(new_n291));
  aoai13aa1n06x5               g196(.a(new_n279), .b(new_n291), .c(new_n290), .d(new_n264), .o1(new_n292));
  tech160nm_fioai012aa1n05x5   g197(.a(new_n281), .b(new_n292), .c(new_n289), .o1(new_n293));
  aoi012aa1n03x5               g198(.a(new_n286), .b(new_n293), .c(new_n284), .o1(new_n294));
  norp02aa1n03x5               g199(.a(new_n294), .b(new_n287), .o1(\s[28] ));
  norb02aa1n02x5               g200(.a(new_n281), .b(new_n286), .out0(new_n296));
  oai012aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n289), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  aoi012aa1n03x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n03x5               g205(.a(new_n296), .b(new_n278), .c(new_n280), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g209(.a(new_n281), .b(new_n299), .c(new_n286), .out0(new_n305));
  oai012aa1n03x5               g210(.a(new_n305), .b(new_n292), .c(new_n289), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[29] ), .b(\a[30] ), .out0(new_n308));
  aoi012aa1n03x5               g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  aobi12aa1n03x5               g214(.a(new_n305), .b(new_n278), .c(new_n280), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n307), .c(new_n308), .out0(new_n311));
  nor002aa1n02x5               g216(.a(new_n309), .b(new_n311), .o1(\s[30] ));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n305), .b(new_n308), .out0(new_n314));
  aobi12aa1n06x5               g219(.a(new_n314), .b(new_n278), .c(new_n280), .out0(new_n315));
  oao003aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n307), .carry(new_n316));
  nano22aa1n03x7               g221(.a(new_n315), .b(new_n313), .c(new_n316), .out0(new_n317));
  tech160nm_fioai012aa1n05x5   g222(.a(new_n314), .b(new_n292), .c(new_n289), .o1(new_n318));
  aoi012aa1n03x5               g223(.a(new_n313), .b(new_n318), .c(new_n316), .o1(new_n319));
  nor002aa1n02x5               g224(.a(new_n319), .b(new_n317), .o1(\s[31] ));
  xnrb03aa1n02x5               g225(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g226(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g229(.a(new_n118), .b(new_n109), .c(new_n135), .d(new_n134), .o1(new_n325));
  obai22aa1n02x7               g230(.a(new_n325), .b(new_n192), .c(new_n189), .d(new_n191), .out0(new_n326));
  nona32aa1n02x4               g231(.a(new_n325), .b(new_n192), .c(new_n191), .d(new_n189), .out0(new_n327));
  nanp02aa1n02x5               g232(.a(new_n326), .b(new_n327), .o1(\s[6] ));
  xnbna2aa1n03x5               g233(.a(new_n124), .b(new_n327), .c(new_n122), .out0(\s[7] ));
  nona22aa1n02x4               g234(.a(new_n327), .b(new_n124), .c(new_n189), .out0(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n114), .b(new_n330), .c(new_n115), .out0(\s[8] ));
  xobna2aa1n03x5               g236(.a(new_n128), .b(new_n120), .c(new_n127), .out0(\s[9] ));
endmodule


