// Benchmark "adder" written by ABC on Wed Jul 17 20:41:25 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n332, new_n333, new_n335, new_n338, new_n339, new_n341,
    new_n342, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  tech160nm_fixnrc02aa1n05x5   g002(.a(\b[3] ), .b(\a[4] ), .out0(new_n98));
  nand42aa1d28x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nand42aa1n06x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanb03aa1n06x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  nand22aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor022aa1n06x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norb03aa1n03x5               g009(.a(new_n99), .b(new_n104), .c(new_n103), .out0(new_n105));
  inv040aa1n03x5               g010(.a(new_n105), .o1(new_n106));
  nona22aa1n09x5               g011(.a(new_n106), .b(new_n102), .c(new_n98), .out0(new_n107));
  inv000aa1n06x5               g012(.a(new_n100), .o1(new_n108));
  oao003aa1n06x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .carry(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  xorc02aa1n12x5               g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1n10x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanb02aa1d24x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  nona23aa1n09x5               g020(.a(new_n111), .b(new_n112), .c(new_n110), .d(new_n115), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n113), .b(new_n117), .c(new_n114), .out0(new_n118));
  oai022aa1n03x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  orn002aa1n02x5               g024(.a(\a[7] ), .b(\b[6] ), .o(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  aoi013aa1n06x4               g026(.a(new_n121), .b(new_n118), .c(new_n112), .d(new_n119), .o1(new_n122));
  aoai13aa1n12x5               g027(.a(new_n122), .b(new_n116), .c(new_n107), .d(new_n109), .o1(new_n123));
  tech160nm_fixnrc02aa1n04x5   g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  aoib12aa1n02x5               g029(.a(new_n97), .b(new_n123), .c(new_n124), .out0(new_n125));
  nor002aa1d24x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nand02aa1d10x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb03aa1n02x5               g033(.a(new_n128), .b(new_n97), .c(new_n126), .out0(new_n129));
  oaib12aa1n02x5               g034(.a(new_n129), .b(new_n124), .c(new_n123), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n125), .c(new_n128), .d(new_n127), .o1(\s[10] ));
  oai013aa1n03x5               g036(.a(new_n109), .b(new_n105), .c(new_n102), .d(new_n98), .o1(new_n132));
  tech160nm_fixorc02aa1n03p5x5 g037(.a(\a[6] ), .b(\b[5] ), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n111), .b(new_n133), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n115), .o1(new_n135));
  nano22aa1n03x7               g040(.a(new_n134), .b(new_n135), .c(new_n112), .out0(new_n136));
  inv040aa1n03x5               g041(.a(new_n122), .o1(new_n137));
  nano22aa1n03x7               g042(.a(new_n124), .b(new_n127), .c(new_n128), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n132), .d(new_n136), .o1(new_n139));
  oai012aa1n02x5               g044(.a(new_n128), .b(new_n126), .c(new_n97), .o1(new_n140));
  tech160nm_fixorc02aa1n03p5x5 g045(.a(\a[11] ), .b(\b[10] ), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  orn002aa1n24x5               g047(.a(\a[11] ), .b(\b[10] ), .o(new_n143));
  aob012aa1n02x5               g048(.a(new_n141), .b(new_n139), .c(new_n140), .out0(new_n144));
  norp02aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand02aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x7               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  oab012aa1n02x4               g052(.a(new_n145), .b(\a[11] ), .c(\b[10] ), .out0(new_n148));
  nanp03aa1n02x5               g053(.a(new_n144), .b(new_n146), .c(new_n148), .o1(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n143), .d(new_n144), .o1(\s[12] ));
  and003aa1n02x5               g055(.a(new_n138), .b(new_n147), .c(new_n141), .o(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n137), .c(new_n132), .d(new_n136), .o1(new_n152));
  nand42aa1n02x5               g057(.a(\b[10] ), .b(\a[11] ), .o1(new_n153));
  nanb03aa1n06x5               g058(.a(new_n145), .b(new_n146), .c(new_n153), .out0(new_n154));
  oai112aa1n06x5               g059(.a(new_n128), .b(new_n143), .c(new_n126), .d(new_n97), .o1(new_n155));
  oaoi03aa1n09x5               g060(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .o1(new_n156));
  inv000aa1n03x5               g061(.a(new_n156), .o1(new_n157));
  oai012aa1n18x5               g062(.a(new_n157), .b(new_n155), .c(new_n154), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nor002aa1n16x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand02aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(new_n162));
  xobna2aa1n03x5               g067(.a(new_n162), .b(new_n152), .c(new_n159), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(new_n160), .o1(new_n164));
  tech160nm_fiao0012aa1n02p5x5 g069(.a(new_n162), .b(new_n152), .c(new_n159), .o(new_n165));
  nor042aa1n03x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand42aa1n08x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  nona23aa1n02x4               g073(.a(new_n165), .b(new_n167), .c(new_n166), .d(new_n160), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n168), .c(new_n164), .d(new_n165), .o1(\s[14] ));
  nano23aa1n09x5               g075(.a(new_n160), .b(new_n166), .c(new_n167), .d(new_n161), .out0(new_n171));
  oai012aa1n02x5               g076(.a(new_n167), .b(new_n166), .c(new_n160), .o1(new_n172));
  aobi12aa1n12x5               g077(.a(new_n172), .b(new_n158), .c(new_n171), .out0(new_n173));
  oaib12aa1n06x5               g078(.a(new_n173), .b(new_n152), .c(new_n171), .out0(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  tech160nm_fixnrc02aa1n05x5   g080(.a(\b[14] ), .b(\a[15] ), .out0(new_n176));
  inv000aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  norp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n178), .c(new_n174), .d(new_n177), .o1(new_n180));
  oai022aa1n02x5               g085(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n181));
  tech160nm_fiao0012aa1n02p5x5 g086(.a(new_n181), .b(\a[16] ), .c(\b[15] ), .o(new_n182));
  aoai13aa1n02x5               g087(.a(new_n180), .b(new_n182), .c(new_n177), .d(new_n174), .o1(\s[16] ));
  nanb02aa1n02x5               g088(.a(new_n179), .b(new_n177), .out0(new_n184));
  nona22aa1n03x5               g089(.a(new_n171), .b(new_n176), .c(new_n179), .out0(new_n185));
  nano32aa1d12x5               g090(.a(new_n185), .b(new_n138), .c(new_n141), .d(new_n147), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n137), .c(new_n132), .d(new_n136), .o1(new_n187));
  aob012aa1n02x5               g092(.a(new_n181), .b(\b[15] ), .c(\a[16] ), .out0(new_n188));
  oai112aa1n06x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .d(new_n173), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nano22aa1n03x5               g097(.a(new_n145), .b(new_n153), .c(new_n146), .out0(new_n193));
  tech160nm_fioai012aa1n03p5x5 g098(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .o1(new_n194));
  oab012aa1n03x5               g099(.a(new_n194), .b(new_n97), .c(new_n126), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n171), .b(new_n156), .c(new_n195), .d(new_n193), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n188), .b(new_n184), .c(new_n196), .d(new_n172), .o1(new_n197));
  tech160nm_fixorc02aa1n04x5   g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n123), .d(new_n186), .o1(new_n199));
  nor002aa1d32x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nand42aa1n06x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  norb02aa1n03x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  oai022aa1n02x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  nanb03aa1n02x5               g108(.a(new_n203), .b(new_n199), .c(new_n201), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n202), .c(new_n192), .d(new_n199), .o1(\s[18] ));
  and002aa1n02x5               g110(.a(new_n198), .b(new_n202), .o(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n197), .c(new_n123), .d(new_n186), .o1(new_n207));
  oaoi03aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  xorc02aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g117(.a(\a[19] ), .o1(new_n213));
  nanb02aa1d24x5               g118(.a(\b[18] ), .b(new_n213), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n210), .b(new_n208), .c(new_n189), .d(new_n206), .o1(new_n215));
  nor042aa1n03x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1d06x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n210), .o1(new_n219));
  nano22aa1n02x4               g124(.a(new_n216), .b(new_n214), .c(new_n217), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n219), .c(new_n207), .d(new_n209), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n218), .c(new_n215), .d(new_n214), .o1(\s[20] ));
  nand42aa1n02x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nano32aa1n03x7               g128(.a(new_n216), .b(new_n214), .c(new_n217), .d(new_n223), .out0(new_n224));
  nand23aa1d12x5               g129(.a(new_n224), .b(new_n198), .c(new_n202), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n197), .c(new_n123), .d(new_n186), .o1(new_n227));
  nano22aa1n03x7               g132(.a(new_n216), .b(new_n223), .c(new_n217), .out0(new_n228));
  tech160nm_fioai012aa1n03p5x5 g133(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .o1(new_n229));
  oab012aa1n03x5               g134(.a(new_n229), .b(new_n191), .c(new_n200), .out0(new_n230));
  oaoi03aa1n02x5               g135(.a(\a[20] ), .b(\b[19] ), .c(new_n214), .o1(new_n231));
  tech160nm_fiao0012aa1n02p5x5 g136(.a(new_n231), .b(new_n230), .c(new_n228), .o(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nor042aa1d18x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nand42aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n227), .c(new_n233), .out0(\s[21] ));
  inv000aa1d42x5               g142(.a(new_n234), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n236), .b(new_n232), .c(new_n189), .d(new_n226), .o1(new_n239));
  nor042aa1n06x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nand02aa1d08x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n236), .o1(new_n243));
  norb03aa1n02x5               g148(.a(new_n241), .b(new_n234), .c(new_n240), .out0(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n243), .c(new_n227), .d(new_n233), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n242), .c(new_n239), .d(new_n238), .o1(\s[22] ));
  nano23aa1n09x5               g151(.a(new_n234), .b(new_n240), .c(new_n241), .d(new_n235), .out0(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n225), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n197), .c(new_n123), .d(new_n186), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n247), .b(new_n231), .c(new_n230), .d(new_n228), .o1(new_n250));
  oai012aa1d24x5               g155(.a(new_n241), .b(new_n240), .c(new_n234), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n250), .b(new_n252), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n249), .c(new_n253), .out0(\s[23] ));
  norp02aa1n02x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n249), .b(new_n253), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n258), .b(new_n254), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n254), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  oai022aa1n02x5               g167(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n262), .b(new_n263), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n249), .d(new_n253), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n260), .c(new_n259), .d(new_n257), .o1(\s[24] ));
  nano32aa1n03x7               g171(.a(new_n225), .b(new_n260), .c(new_n247), .d(new_n254), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n197), .c(new_n123), .d(new_n186), .o1(new_n268));
  and002aa1n03x5               g173(.a(new_n260), .b(new_n254), .o(new_n269));
  inv000aa1n06x5               g174(.a(new_n269), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n263), .b(new_n262), .o1(new_n271));
  aoai13aa1n12x5               g176(.a(new_n271), .b(new_n270), .c(new_n250), .d(new_n251), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n268), .c(new_n273), .out0(\s[25] ));
  norp02aa1n02x5               g180(.a(\b[24] ), .b(\a[25] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n274), .b(new_n272), .c(new_n189), .d(new_n267), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n274), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .o1(new_n281));
  oai022aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n282));
  norb02aa1n02x5               g187(.a(new_n281), .b(new_n282), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n283), .b(new_n280), .c(new_n268), .d(new_n273), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n279), .c(new_n278), .d(new_n277), .o1(\s[26] ));
  and002aa1n12x5               g190(.a(new_n279), .b(new_n274), .o(new_n286));
  nano23aa1n06x5               g191(.a(new_n270), .b(new_n225), .c(new_n286), .d(new_n247), .out0(new_n287));
  nanp02aa1n09x5               g192(.a(new_n272), .b(new_n286), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n282), .b(new_n281), .o1(new_n289));
  nanp02aa1n06x5               g194(.a(new_n288), .b(new_n289), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n290), .c(new_n189), .d(new_n287), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n287), .b(new_n197), .c(new_n123), .d(new_n186), .o1(new_n293));
  nano32aa1n02x4               g198(.a(new_n291), .b(new_n293), .c(new_n288), .d(new_n289), .out0(new_n294));
  norb02aa1n03x4               g199(.a(new_n292), .b(new_n294), .out0(\s[27] ));
  norp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .out0(new_n298));
  aoi022aa1d18x5               g203(.a(new_n272), .b(new_n286), .c(new_n281), .d(new_n282), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n291), .o1(new_n300));
  oai022aa1n02x5               g205(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(\a[28] ), .c(\b[27] ), .o1(new_n302));
  aoai13aa1n02x7               g207(.a(new_n302), .b(new_n300), .c(new_n293), .d(new_n299), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n298), .c(new_n292), .d(new_n297), .o1(\s[28] ));
  and002aa1n02x5               g209(.a(new_n298), .b(new_n291), .o(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  aob012aa1n02x5               g211(.a(new_n301), .b(\b[27] ), .c(\a[28] ), .out0(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n307), .b(new_n308), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n306), .c(new_n293), .d(new_n299), .o1(new_n310));
  aoai13aa1n06x5               g215(.a(new_n307), .b(new_n306), .c(new_n293), .d(new_n299), .o1(new_n311));
  nanp02aa1n03x5               g216(.a(new_n311), .b(new_n308), .o1(new_n312));
  nanp02aa1n03x5               g217(.a(new_n312), .b(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g219(.a(new_n308), .b(new_n291), .c(new_n298), .out0(new_n315));
  oao003aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .carry(new_n316));
  aoai13aa1n06x5               g221(.a(new_n316), .b(new_n315), .c(new_n293), .d(new_n299), .o1(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  nanp02aa1n03x5               g223(.a(new_n317), .b(new_n318), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n316), .b(new_n318), .out0(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n315), .c(new_n293), .d(new_n299), .o1(new_n321));
  nanp02aa1n03x5               g226(.a(new_n319), .b(new_n321), .o1(\s[30] ));
  nano23aa1n09x5               g227(.a(new_n318), .b(new_n308), .c(new_n298), .d(new_n291), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n290), .c(new_n189), .d(new_n287), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  inv000aa1d42x5               g230(.a(new_n323), .o1(new_n326));
  oai012aa1n02x5               g231(.a(new_n325), .b(\b[29] ), .c(\a[30] ), .o1(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n316), .c(new_n318), .out0(new_n328));
  aoai13aa1n02x7               g233(.a(new_n328), .b(new_n326), .c(new_n293), .d(new_n299), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n330));
  aoai13aa1n03x5               g235(.a(new_n329), .b(new_n325), .c(new_n324), .d(new_n330), .o1(\s[31] ));
  norp02aa1n02x5               g236(.a(new_n105), .b(new_n102), .o1(new_n332));
  aboi22aa1n03x5               g237(.a(new_n105), .b(new_n99), .c(new_n108), .d(new_n101), .out0(new_n333));
  norp02aa1n02x5               g238(.a(new_n333), .b(new_n332), .o1(\s[3] ));
  inv000aa1n03x5               g239(.a(new_n332), .o1(new_n335));
  xobna2aa1n03x5               g240(.a(new_n98), .b(new_n335), .c(new_n108), .out0(\s[4] ));
  xnbna2aa1n03x5               g241(.a(new_n111), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  orn002aa1n02x5               g242(.a(\a[5] ), .b(\b[4] ), .o(new_n338));
  nanp02aa1n02x5               g243(.a(new_n132), .b(new_n111), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n133), .b(new_n339), .c(new_n338), .out0(\s[6] ));
  nanp02aa1n02x5               g245(.a(new_n119), .b(new_n117), .o1(new_n341));
  aoai13aa1n02x5               g246(.a(new_n341), .b(new_n134), .c(new_n107), .d(new_n109), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp02aa1n02x5               g248(.a(new_n342), .b(new_n135), .o1(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n112), .b(new_n344), .c(new_n120), .out0(\s[8] ));
  xorb03aa1n02x5               g250(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


