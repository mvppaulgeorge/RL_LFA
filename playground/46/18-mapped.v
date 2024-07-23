// Benchmark "adder" written by ABC on Thu Jul 18 11:42:30 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n187, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n340, new_n342, new_n343, new_n346, new_n347,
    new_n349, new_n350, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanp02aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanb03aa1n06x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  nand22aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norb03aa1n12x5               g009(.a(new_n101), .b(new_n104), .c(new_n103), .out0(new_n105));
  tech160nm_fixnrc02aa1n04x5   g010(.a(\b[3] ), .b(\a[4] ), .out0(new_n106));
  orn002aa1n03x5               g011(.a(\a[3] ), .b(\b[2] ), .o(new_n107));
  oao003aa1n09x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  oai013aa1n09x5               g013(.a(new_n108), .b(new_n105), .c(new_n102), .d(new_n106), .o1(new_n109));
  xorc02aa1n02x5               g014(.a(\a[6] ), .b(\b[5] ), .out0(new_n110));
  tech160nm_fixorc02aa1n02p5x5 g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  nor002aa1n08x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norb02aa1n06x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nand02aa1n02x5               g020(.a(new_n112), .b(new_n115), .o1(new_n116));
  nano22aa1n03x7               g021(.a(new_n116), .b(new_n110), .c(new_n111), .out0(new_n117));
  tech160nm_finand02aa1n03p5x5 g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nano22aa1n03x7               g023(.a(new_n113), .b(new_n118), .c(new_n114), .out0(new_n119));
  oai022aa1d18x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  nand43aa1n02x5               g025(.a(new_n119), .b(new_n112), .c(new_n120), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  aob012aa1n02x5               g027(.a(new_n113), .b(\b[7] ), .c(\a[8] ), .out0(new_n123));
  oaib12aa1n02x5               g028(.a(new_n123), .b(\b[7] ), .c(new_n122), .out0(new_n124));
  nanb02aa1n06x5               g029(.a(new_n124), .b(new_n121), .out0(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n109), .d(new_n117), .o1(new_n127));
  nor042aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1d28x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n12x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  nano22aa1n03x7               g036(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n132));
  xorc02aa1n12x5               g037(.a(\a[4] ), .b(\b[3] ), .out0(new_n133));
  nanb03aa1n12x5               g038(.a(new_n105), .b(new_n133), .c(new_n132), .out0(new_n134));
  xnrc02aa1n02x5               g039(.a(\b[5] ), .b(\a[6] ), .out0(new_n135));
  xnrc02aa1n02x5               g040(.a(\b[4] ), .b(\a[5] ), .out0(new_n136));
  nona23aa1n08x5               g041(.a(new_n112), .b(new_n115), .c(new_n136), .d(new_n135), .out0(new_n137));
  aoi013aa1n06x4               g042(.a(new_n124), .b(new_n119), .c(new_n112), .d(new_n120), .o1(new_n138));
  aoai13aa1n12x5               g043(.a(new_n138), .b(new_n137), .c(new_n134), .d(new_n108), .o1(new_n139));
  aoai13aa1n06x5               g044(.a(new_n130), .b(new_n97), .c(new_n139), .d(new_n126), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n129), .o1(new_n141));
  oai022aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n142));
  oaib12aa1n06x5               g047(.a(new_n140), .b(new_n141), .c(new_n142), .out0(new_n143));
  nor042aa1n06x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nand42aa1n20x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  aboi22aa1n03x5               g051(.a(new_n144), .b(new_n145), .c(new_n142), .d(new_n129), .out0(new_n147));
  aoi022aa1n02x5               g052(.a(new_n143), .b(new_n146), .c(new_n140), .d(new_n147), .o1(\s[11] ));
  nor022aa1n08x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand42aa1n16x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n03x5               g057(.a(new_n152), .b(new_n144), .c(new_n143), .d(new_n145), .o1(new_n153));
  aobi12aa1n03x5               g058(.a(new_n130), .b(new_n127), .c(new_n98), .out0(new_n154));
  aoai13aa1n02x5               g059(.a(new_n146), .b(new_n154), .c(new_n129), .d(new_n142), .o1(new_n155));
  nona22aa1n02x5               g060(.a(new_n155), .b(new_n152), .c(new_n144), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n153), .b(new_n156), .o1(\s[12] ));
  nano23aa1n06x5               g062(.a(new_n144), .b(new_n149), .c(new_n150), .d(new_n145), .out0(new_n158));
  nand23aa1d12x5               g063(.a(new_n158), .b(new_n126), .c(new_n130), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n125), .c(new_n109), .d(new_n117), .o1(new_n161));
  nano22aa1n03x5               g066(.a(new_n149), .b(new_n145), .c(new_n150), .out0(new_n162));
  oai012aa1n02x5               g067(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .o1(new_n163));
  oab012aa1n03x5               g068(.a(new_n163), .b(new_n97), .c(new_n128), .out0(new_n164));
  nanp02aa1n02x5               g069(.a(new_n164), .b(new_n162), .o1(new_n165));
  aoi012aa1n02x7               g070(.a(new_n149), .b(new_n144), .c(new_n150), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n165), .b(new_n166), .o1(new_n167));
  inv020aa1n02x5               g072(.a(new_n167), .o1(new_n168));
  nor042aa1n06x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n161), .c(new_n168), .out0(\s[13] ));
  nanp02aa1n03x5               g077(.a(new_n161), .b(new_n168), .o1(new_n173));
  tech160nm_fiaoi012aa1n05x5   g078(.a(new_n169), .b(new_n173), .c(new_n170), .o1(new_n174));
  xnrb03aa1n03x5               g079(.a(new_n174), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g080(.a(\b[13] ), .b(\a[14] ), .o1(new_n176));
  nand42aa1d28x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nano23aa1d15x5               g082(.a(new_n169), .b(new_n176), .c(new_n177), .d(new_n170), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  aoi012aa1d18x5               g084(.a(new_n176), .b(new_n169), .c(new_n177), .o1(new_n180));
  aoai13aa1n06x5               g085(.a(new_n180), .b(new_n179), .c(new_n161), .d(new_n168), .o1(new_n181));
  xnrc02aa1n12x5               g086(.a(\b[14] ), .b(\a[15] ), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n180), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n173), .d(new_n178), .o1(new_n185));
  aoi012aa1n02x5               g090(.a(new_n185), .b(new_n181), .c(new_n183), .o1(\s[15] ));
  norp02aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  tech160nm_fixnrc02aa1n02p5x5 g092(.a(\b[15] ), .b(\a[16] ), .out0(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n181), .d(new_n183), .o1(new_n189));
  aoi112aa1n03x4               g094(.a(new_n187), .b(new_n188), .c(new_n181), .d(new_n183), .o1(new_n190));
  nanb02aa1n03x5               g095(.a(new_n190), .b(new_n189), .out0(\s[16] ));
  nor042aa1n06x5               g096(.a(new_n188), .b(new_n182), .o1(new_n192));
  nano22aa1d15x5               g097(.a(new_n159), .b(new_n192), .c(new_n178), .out0(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n125), .c(new_n109), .d(new_n117), .o1(new_n194));
  inv020aa1n02x5               g099(.a(new_n166), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n178), .b(new_n195), .c(new_n164), .d(new_n162), .o1(new_n196));
  nanp02aa1n03x5               g101(.a(new_n196), .b(new_n180), .o1(new_n197));
  nand02aa1n04x5               g102(.a(new_n197), .b(new_n192), .o1(new_n198));
  orn002aa1n02x5               g103(.a(\a[15] ), .b(\b[14] ), .o(new_n199));
  oao003aa1n02x5               g104(.a(\a[16] ), .b(\b[15] ), .c(new_n199), .carry(new_n200));
  nand23aa1d12x5               g105(.a(new_n194), .b(new_n198), .c(new_n200), .o1(new_n201));
  nor002aa1d32x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  nand42aa1n16x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nano22aa1n02x4               g109(.a(new_n204), .b(new_n198), .c(new_n200), .out0(new_n205));
  aoi022aa1n02x5               g110(.a(new_n205), .b(new_n194), .c(new_n201), .d(new_n204), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g111(.a(new_n202), .b(new_n201), .c(new_n204), .o1(new_n207));
  xnrb03aa1n03x5               g112(.a(new_n207), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1n02x5               g113(.a(new_n192), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n200), .b(new_n209), .c(new_n196), .d(new_n180), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nand02aa1d28x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nano23aa1d15x5               g117(.a(new_n202), .b(new_n211), .c(new_n212), .d(new_n203), .out0(new_n213));
  aoai13aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n139), .d(new_n193), .o1(new_n214));
  oa0012aa1n02x5               g119(.a(new_n212), .b(new_n211), .c(new_n202), .o(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nor042aa1d18x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nanp02aa1n12x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n09x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n214), .c(new_n216), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g126(.a(new_n214), .b(new_n216), .o1(new_n222));
  nor002aa1n16x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand02aa1d28x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanb02aa1n06x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n217), .c(new_n222), .d(new_n218), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n219), .b(new_n215), .c(new_n201), .d(new_n213), .o1(new_n227));
  nona22aa1n02x5               g132(.a(new_n227), .b(new_n225), .c(new_n217), .out0(new_n228));
  nanp02aa1n03x5               g133(.a(new_n226), .b(new_n228), .o1(\s[20] ));
  nanb03aa1n12x5               g134(.a(new_n223), .b(new_n224), .c(new_n218), .out0(new_n230));
  oai122aa1n12x5               g135(.a(new_n212), .b(new_n211), .c(new_n202), .d(\b[18] ), .e(\a[19] ), .o1(new_n231));
  aoi012aa1d18x5               g136(.a(new_n223), .b(new_n217), .c(new_n224), .o1(new_n232));
  oai012aa1d24x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aobi12aa1n02x5               g139(.a(new_n200), .b(new_n197), .c(new_n192), .out0(new_n235));
  nanb03aa1d18x5               g140(.a(new_n225), .b(new_n213), .c(new_n219), .out0(new_n236));
  aoai13aa1n02x7               g141(.a(new_n234), .b(new_n236), .c(new_n235), .d(new_n194), .o1(new_n237));
  nor042aa1n09x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  nand42aa1n08x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n236), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n240), .b(new_n201), .c(new_n241), .o1(new_n242));
  aoi022aa1n02x5               g147(.a(new_n242), .b(new_n234), .c(new_n237), .d(new_n240), .o1(\s[21] ));
  nor042aa1n06x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nand42aa1n16x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nanb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n238), .c(new_n237), .d(new_n240), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n240), .b(new_n233), .c(new_n201), .d(new_n241), .o1(new_n248));
  nona22aa1n02x5               g153(.a(new_n248), .b(new_n246), .c(new_n238), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n247), .b(new_n249), .o1(\s[22] ));
  nano23aa1d15x5               g155(.a(new_n238), .b(new_n244), .c(new_n245), .d(new_n239), .out0(new_n251));
  nano32aa1n02x4               g156(.a(new_n225), .b(new_n251), .c(new_n213), .d(new_n219), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n210), .c(new_n139), .d(new_n193), .o1(new_n253));
  aoi012aa1d18x5               g158(.a(new_n244), .b(new_n238), .c(new_n245), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n233), .c(new_n251), .o1(new_n256));
  nand42aa1n04x5               g161(.a(new_n253), .b(new_n256), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n258), .b(new_n255), .c(new_n233), .d(new_n251), .o1(new_n259));
  aoi022aa1n02x5               g164(.a(new_n257), .b(new_n258), .c(new_n253), .d(new_n259), .o1(\s[23] ));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  xnrc02aa1n12x5               g166(.a(\b[23] ), .b(\a[24] ), .out0(new_n262));
  aoai13aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n257), .d(new_n258), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n256), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n258), .b(new_n264), .c(new_n201), .d(new_n252), .o1(new_n265));
  nona22aa1n03x5               g170(.a(new_n265), .b(new_n262), .c(new_n261), .out0(new_n266));
  nanp02aa1n03x5               g171(.a(new_n263), .b(new_n266), .o1(\s[24] ));
  norb02aa1n09x5               g172(.a(new_n258), .b(new_n262), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n236), .b(new_n268), .c(new_n251), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n210), .c(new_n139), .d(new_n193), .o1(new_n270));
  nano22aa1n03x5               g175(.a(new_n223), .b(new_n218), .c(new_n224), .out0(new_n271));
  oai012aa1n02x7               g176(.a(new_n212), .b(\b[18] ), .c(\a[19] ), .o1(new_n272));
  oab012aa1n03x5               g177(.a(new_n272), .b(new_n202), .c(new_n211), .out0(new_n273));
  inv020aa1n03x5               g178(.a(new_n232), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n251), .b(new_n274), .c(new_n273), .d(new_n271), .o1(new_n275));
  inv040aa1n02x5               g180(.a(new_n268), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n277));
  oab012aa1n04x5               g182(.a(new_n277), .b(\a[24] ), .c(\b[23] ), .out0(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n276), .c(new_n275), .d(new_n254), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  tech160nm_finand02aa1n05x5   g185(.a(new_n270), .b(new_n280), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n268), .b(new_n255), .c(new_n233), .d(new_n251), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n282), .b(new_n283), .c(new_n278), .out0(new_n284));
  aoi022aa1n02x5               g189(.a(new_n281), .b(new_n282), .c(new_n270), .d(new_n284), .o1(\s[25] ));
  nor002aa1n02x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  tech160nm_fixnrc02aa1n05x5   g191(.a(\b[25] ), .b(\a[26] ), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n281), .d(new_n282), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n282), .b(new_n279), .c(new_n201), .d(new_n269), .o1(new_n289));
  nona22aa1n03x5               g194(.a(new_n289), .b(new_n287), .c(new_n286), .out0(new_n290));
  nanp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[26] ));
  norb02aa1n12x5               g196(.a(new_n282), .b(new_n287), .out0(new_n292));
  inv020aa1n02x5               g197(.a(new_n292), .o1(new_n293));
  nano23aa1d12x5               g198(.a(new_n293), .b(new_n236), .c(new_n268), .d(new_n251), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n210), .c(new_n139), .d(new_n193), .o1(new_n295));
  inv040aa1d32x5               g200(.a(\a[26] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\b[25] ), .o1(new_n297));
  tech160nm_fioaoi03aa1n03p5x5 g202(.a(new_n296), .b(new_n297), .c(new_n286), .o1(new_n298));
  aoai13aa1n04x5               g203(.a(new_n298), .b(new_n293), .c(new_n283), .d(new_n278), .o1(new_n299));
  xorc02aa1n12x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n299), .c(new_n201), .d(new_n294), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n298), .o1(new_n302));
  aoi112aa1n02x5               g207(.a(new_n300), .b(new_n302), .c(new_n279), .d(new_n292), .o1(new_n303));
  aobi12aa1n02x7               g208(.a(new_n301), .b(new_n303), .c(new_n295), .out0(\s[27] ));
  xnrc02aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .out0(new_n305));
  aoi012aa1n09x5               g210(.a(new_n302), .b(new_n279), .c(new_n292), .o1(new_n306));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  inv000aa1n03x5               g212(.a(new_n307), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n300), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n309), .c(new_n295), .d(new_n306), .o1(new_n310));
  nand02aa1n02x5               g215(.a(new_n310), .b(new_n305), .o1(new_n311));
  nona22aa1n02x5               g216(.a(new_n301), .b(new_n305), .c(new_n307), .out0(new_n312));
  nanp02aa1n03x5               g217(.a(new_n311), .b(new_n312), .o1(\s[28] ));
  norb02aa1n09x5               g218(.a(new_n300), .b(new_n305), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n299), .c(new_n201), .d(new_n294), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n314), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n295), .d(new_n306), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n317), .b(new_n319), .out0(new_n320));
  aoi022aa1n03x5               g225(.a(new_n318), .b(new_n319), .c(new_n315), .d(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g227(.a(new_n305), .b(new_n300), .c(new_n319), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n299), .c(new_n201), .d(new_n294), .o1(new_n324));
  inv000aa1n02x5               g229(.a(new_n323), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n295), .d(new_n306), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  norb02aa1n02x5               g233(.a(new_n326), .b(new_n328), .out0(new_n329));
  aoi022aa1n02x7               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[30] ));
  nano22aa1n06x5               g235(.a(new_n316), .b(new_n319), .c(new_n328), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n299), .c(new_n201), .d(new_n294), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n334));
  norb02aa1n02x5               g239(.a(new_n334), .b(new_n333), .out0(new_n335));
  inv000aa1d42x5               g240(.a(new_n331), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n334), .b(new_n336), .c(new_n295), .d(new_n306), .o1(new_n337));
  aoi022aa1n02x7               g242(.a(new_n337), .b(new_n333), .c(new_n332), .d(new_n335), .o1(\s[31] ));
  oai012aa1n02x5               g243(.a(new_n101), .b(new_n104), .c(new_n103), .o1(new_n339));
  nanb02aa1n02x5               g244(.a(new_n100), .b(new_n99), .out0(new_n340));
  aboi22aa1n03x5               g245(.a(new_n105), .b(new_n132), .c(new_n340), .d(new_n339), .out0(\s[3] ));
  norp02aa1n02x5               g246(.a(new_n105), .b(new_n102), .o1(new_n342));
  norp03aa1n02x5               g247(.a(new_n342), .b(new_n133), .c(new_n100), .o1(new_n343));
  oaoi13aa1n02x5               g248(.a(new_n343), .b(new_n109), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g249(.a(new_n111), .b(new_n134), .c(new_n108), .out0(\s[5] ));
  orn002aa1n02x5               g250(.a(\a[5] ), .b(\b[4] ), .o(new_n346));
  aoai13aa1n02x5               g251(.a(new_n346), .b(new_n136), .c(new_n134), .d(new_n108), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nona22aa1n02x4               g253(.a(new_n109), .b(new_n135), .c(new_n136), .out0(new_n349));
  aob012aa1n02x5               g254(.a(new_n349), .b(new_n120), .c(new_n118), .out0(new_n350));
  xorb03aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g256(.a(new_n113), .b(new_n350), .c(new_n114), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[7] ), .c(new_n122), .out0(\s[8] ));
  aoi112aa1n02x5               g258(.a(new_n125), .b(new_n126), .c(new_n109), .d(new_n117), .o1(new_n354));
  aoi012aa1n02x5               g259(.a(new_n354), .b(new_n139), .c(new_n126), .o1(\s[9] ));
endmodule


