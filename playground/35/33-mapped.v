// Benchmark "adder" written by ABC on Thu Jul 18 06:11:24 2024

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
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n338,
    new_n339, new_n340, new_n341, new_n344, new_n345, new_n346, new_n347,
    new_n349, new_n351, new_n352, new_n353, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv040aa1d28x5               g002(.a(\a[4] ), .o1(new_n98));
  inv030aa1d32x5               g003(.a(\b[3] ), .o1(new_n99));
  aoi022aa1n06x5               g004(.a(new_n99), .b(new_n98), .c(\a[2] ), .d(\b[1] ), .o1(new_n100));
  and002aa1n03x5               g005(.a(\b[2] ), .b(\a[3] ), .o(new_n101));
  oai022aa1n06x5               g006(.a(new_n98), .b(new_n99), .c(\a[3] ), .d(\b[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand02aa1d04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nona22aa1n03x5               g010(.a(new_n103), .b(new_n105), .c(new_n104), .out0(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n100), .c(new_n102), .d(new_n101), .out0(new_n107));
  nor042aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  tech160nm_fioaoi03aa1n03p5x5 g013(.a(new_n98), .b(new_n99), .c(new_n108), .o1(new_n109));
  nanp02aa1n09x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand42aa1n10x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nano23aa1n03x7               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  inv000aa1d42x5               g020(.a(new_n115), .o1(new_n116));
  xorc02aa1n12x5               g021(.a(\a[8] ), .b(\b[7] ), .out0(new_n117));
  nand23aa1n02x5               g022(.a(new_n116), .b(new_n114), .c(new_n117), .o1(new_n118));
  nano22aa1n02x5               g023(.a(new_n111), .b(new_n110), .c(new_n113), .out0(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n111), .b(\b[7] ), .c(\a[8] ), .out0(new_n121));
  oai012aa1n02x5               g026(.a(new_n121), .b(\b[7] ), .c(\a[8] ), .o1(new_n122));
  aoi013aa1n06x4               g027(.a(new_n122), .b(new_n119), .c(new_n117), .d(new_n120), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n118), .c(new_n107), .d(new_n109), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  orn002aa1n24x5               g030(.a(\a[10] ), .b(\b[9] ), .o(new_n126));
  nand42aa1n08x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand22aa1n02x5               g032(.a(new_n126), .b(new_n127), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n97), .c(new_n124), .d(new_n125), .o1(new_n129));
  nanp02aa1n03x5               g034(.a(new_n124), .b(new_n125), .o1(new_n130));
  oai112aa1n06x5               g035(.a(new_n126), .b(new_n127), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nanp02aa1n03x5               g037(.a(new_n130), .b(new_n132), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n129), .b(new_n133), .o1(\s[10] ));
  nanp02aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\b[10] ), .o1(new_n136));
  nanb02aa1d36x5               g041(.a(\a[11] ), .b(new_n136), .out0(new_n137));
  nanp03aa1n02x5               g042(.a(new_n137), .b(new_n127), .c(new_n135), .o1(new_n138));
  nand02aa1n02x5               g043(.a(new_n137), .b(new_n135), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n127), .b(new_n131), .c(new_n124), .d(new_n125), .o1(new_n140));
  aboi22aa1n03x5               g045(.a(new_n138), .b(new_n133), .c(new_n140), .d(new_n139), .out0(\s[11] ));
  norp02aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanb02aa1n06x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  inv040aa1n02x5               g049(.a(new_n144), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n137), .o1(new_n146));
  aoi113aa1n02x5               g051(.a(new_n145), .b(new_n146), .c(new_n133), .d(new_n135), .e(new_n127), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n137), .b(new_n138), .c(new_n130), .d(new_n132), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(\s[12] ));
  nona23aa1d18x5               g054(.a(new_n145), .b(new_n125), .c(new_n128), .d(new_n139), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nano32aa1n02x5               g056(.a(new_n144), .b(new_n137), .c(new_n135), .d(new_n127), .out0(new_n152));
  oaoi03aa1n12x5               g057(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aob012aa1n02x5               g059(.a(new_n154), .b(new_n152), .c(new_n131), .out0(new_n155));
  nor002aa1d24x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  tech160nm_finand02aa1n03p5x5 g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  aoai13aa1n03x5               g063(.a(new_n158), .b(new_n155), .c(new_n124), .d(new_n151), .o1(new_n159));
  aoi112aa1n02x5               g064(.a(new_n158), .b(new_n155), .c(new_n124), .d(new_n151), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(\s[13] ));
  inv000aa1d42x5               g066(.a(new_n156), .o1(new_n162));
  nor022aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand02aa1n06x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n159), .c(new_n162), .out0(\s[14] ));
  nona23aa1d24x5               g071(.a(new_n164), .b(new_n157), .c(new_n156), .d(new_n163), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n155), .c(new_n124), .d(new_n151), .o1(new_n169));
  oaoi03aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n162), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  nor002aa1n10x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n12x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n169), .c(new_n171), .out0(\s[15] ));
  aob012aa1n03x5               g080(.a(new_n174), .b(new_n169), .c(new_n171), .out0(new_n176));
  norp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n06x4               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  aoib12aa1n02x5               g084(.a(new_n172), .b(new_n178), .c(new_n177), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n172), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n174), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n169), .d(new_n171), .o1(new_n183));
  aoi022aa1n03x5               g088(.a(new_n183), .b(new_n179), .c(new_n176), .d(new_n180), .o1(\s[16] ));
  nano22aa1n03x7               g089(.a(new_n167), .b(new_n174), .c(new_n179), .out0(new_n185));
  norb02aa1n09x5               g090(.a(new_n185), .b(new_n150), .out0(new_n186));
  nand02aa1d06x5               g091(.a(new_n124), .b(new_n186), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n185), .b(new_n153), .c(new_n152), .d(new_n131), .o1(new_n188));
  oai112aa1n03x5               g093(.a(new_n164), .b(new_n173), .c(new_n163), .d(new_n156), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n181), .o1(new_n190));
  aoi012aa1n02x7               g095(.a(new_n177), .b(new_n190), .c(new_n178), .o1(new_n191));
  nand22aa1n06x5               g096(.a(new_n188), .b(new_n191), .o1(new_n192));
  inv000aa1n06x5               g097(.a(new_n192), .o1(new_n193));
  nand02aa1d08x5               g098(.a(new_n187), .b(new_n193), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  nano22aa1n02x4               g100(.a(new_n195), .b(new_n188), .c(new_n191), .out0(new_n196));
  aoi022aa1n02x5               g101(.a(new_n194), .b(new_n195), .c(new_n187), .d(new_n196), .o1(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(\b[16] ), .b(new_n198), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n195), .b(new_n192), .c(new_n124), .d(new_n186), .o1(new_n200));
  nor002aa1n04x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand02aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  norb02aa1n06x4               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n200), .c(new_n199), .out0(\s[18] ));
  and002aa1n02x5               g109(.a(new_n195), .b(new_n203), .o(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n192), .c(new_n124), .d(new_n186), .o1(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  nor002aa1n20x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand02aa1d04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  norb02aa1d21x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n206), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g118(.a(new_n211), .b(new_n207), .c(new_n194), .d(new_n205), .o1(new_n214));
  nor002aa1n06x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand02aa1d06x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n06x4               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  inv000aa1d42x5               g122(.a(\a[19] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\b[18] ), .o1(new_n219));
  aboi22aa1n03x5               g124(.a(new_n215), .b(new_n216), .c(new_n218), .d(new_n219), .out0(new_n220));
  inv020aa1n02x5               g125(.a(new_n209), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n211), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n221), .b(new_n222), .c(new_n206), .d(new_n208), .o1(new_n223));
  aoi022aa1n03x5               g128(.a(new_n223), .b(new_n217), .c(new_n214), .d(new_n220), .o1(\s[20] ));
  nano32aa1n03x7               g129(.a(new_n222), .b(new_n195), .c(new_n217), .d(new_n203), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n192), .c(new_n124), .d(new_n186), .o1(new_n226));
  nanb03aa1n12x5               g131(.a(new_n215), .b(new_n216), .c(new_n210), .out0(new_n227));
  nor042aa1n02x5               g132(.a(\b[16] ), .b(\a[17] ), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n221), .b(new_n202), .c(new_n201), .d(new_n228), .o1(new_n229));
  aoi012aa1d18x5               g134(.a(new_n215), .b(new_n209), .c(new_n216), .o1(new_n230));
  oai012aa1d24x5               g135(.a(new_n230), .b(new_n229), .c(new_n227), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  nor042aa1n06x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  norb02aa1n03x5               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n226), .c(new_n232), .out0(\s[21] ));
  aoai13aa1n06x5               g141(.a(new_n235), .b(new_n231), .c(new_n194), .d(new_n225), .o1(new_n237));
  nor042aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nand42aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoib12aa1n02x5               g145(.a(new_n233), .b(new_n239), .c(new_n238), .out0(new_n241));
  inv000aa1n03x5               g146(.a(new_n233), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n235), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n242), .b(new_n243), .c(new_n226), .d(new_n232), .o1(new_n244));
  aoi022aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n237), .d(new_n241), .o1(\s[22] ));
  inv020aa1n02x5               g150(.a(new_n225), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n246), .b(new_n235), .c(new_n240), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n192), .c(new_n124), .d(new_n186), .o1(new_n248));
  nano23aa1n06x5               g153(.a(new_n233), .b(new_n238), .c(new_n239), .d(new_n234), .out0(new_n249));
  oaoi03aa1n12x5               g154(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n250));
  tech160nm_fiaoi012aa1n05x5   g155(.a(new_n250), .b(new_n231), .c(new_n249), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n248), .c(new_n251), .out0(\s[23] ));
  inv000aa1d42x5               g158(.a(new_n251), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n252), .b(new_n254), .c(new_n194), .d(new_n247), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .out0(new_n256));
  nor042aa1n06x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n256), .b(new_n257), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n257), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n252), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n259), .b(new_n260), .c(new_n248), .d(new_n251), .o1(new_n261));
  aoi022aa1n03x5               g166(.a(new_n261), .b(new_n256), .c(new_n255), .d(new_n258), .o1(\s[24] ));
  and002aa1n12x5               g167(.a(new_n256), .b(new_n252), .o(new_n263));
  nano22aa1n02x5               g168(.a(new_n246), .b(new_n263), .c(new_n249), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n192), .c(new_n124), .d(new_n186), .o1(new_n265));
  nano22aa1n03x7               g170(.a(new_n215), .b(new_n210), .c(new_n216), .out0(new_n266));
  oai012aa1n06x5               g171(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .o1(new_n267));
  oab012aa1n06x5               g172(.a(new_n267), .b(new_n228), .c(new_n201), .out0(new_n268));
  inv030aa1n02x5               g173(.a(new_n230), .o1(new_n269));
  aoai13aa1n04x5               g174(.a(new_n249), .b(new_n269), .c(new_n268), .d(new_n266), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n250), .o1(new_n271));
  inv000aa1n03x5               g176(.a(new_n263), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n273));
  aoai13aa1n12x5               g178(.a(new_n273), .b(new_n272), .c(new_n270), .d(new_n271), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  nanp02aa1n06x5               g180(.a(new_n265), .b(new_n275), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n263), .b(new_n250), .c(new_n231), .d(new_n249), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n277), .o1(new_n279));
  and003aa1n02x5               g184(.a(new_n278), .b(new_n279), .c(new_n273), .o(new_n280));
  aoi022aa1n02x5               g185(.a(new_n276), .b(new_n277), .c(new_n265), .d(new_n280), .o1(\s[25] ));
  nanp02aa1n03x5               g186(.a(new_n276), .b(new_n277), .o1(new_n282));
  tech160nm_fixorc02aa1n02p5x5 g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  nor042aa1n03x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n283), .b(new_n284), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n284), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n279), .c(new_n265), .d(new_n275), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n287), .b(new_n283), .c(new_n282), .d(new_n285), .o1(\s[26] ));
  and002aa1n12x5               g193(.a(new_n283), .b(new_n277), .o(new_n289));
  nano32aa1n03x7               g194(.a(new_n246), .b(new_n289), .c(new_n249), .d(new_n263), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n192), .c(new_n124), .d(new_n186), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  aoi012aa1n06x5               g198(.a(new_n293), .b(new_n274), .c(new_n289), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(new_n294), .b(new_n291), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  aoi112aa1n02x5               g201(.a(new_n296), .b(new_n293), .c(new_n274), .d(new_n289), .o1(new_n297));
  aoi022aa1n02x5               g202(.a(new_n295), .b(new_n296), .c(new_n291), .d(new_n297), .o1(\s[27] ));
  inv000aa1d42x5               g203(.a(new_n289), .o1(new_n299));
  aoai13aa1n04x5               g204(.a(new_n292), .b(new_n299), .c(new_n278), .d(new_n273), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n296), .b(new_n300), .c(new_n194), .d(new_n290), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n302), .b(new_n303), .o1(new_n304));
  inv000aa1n03x5               g209(.a(new_n303), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n296), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n306), .c(new_n294), .d(new_n291), .o1(new_n307));
  aoi022aa1n03x5               g212(.a(new_n307), .b(new_n302), .c(new_n301), .d(new_n304), .o1(\s[28] ));
  and002aa1n02x5               g213(.a(new_n302), .b(new_n296), .o(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n300), .c(new_n194), .d(new_n290), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n309), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n294), .d(new_n291), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .out0(new_n314));
  norb02aa1n02x5               g219(.a(new_n312), .b(new_n314), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n313), .b(new_n314), .c(new_n310), .d(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g222(.a(new_n306), .b(new_n302), .c(new_n314), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n300), .c(new_n194), .d(new_n290), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n318), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n320), .c(new_n294), .d(new_n291), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n321), .b(new_n323), .out0(new_n324));
  aoi022aa1n03x5               g229(.a(new_n322), .b(new_n323), .c(new_n319), .d(new_n324), .o1(\s[30] ));
  nano32aa1n03x7               g230(.a(new_n306), .b(new_n323), .c(new_n302), .d(new_n314), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n300), .c(new_n194), .d(new_n290), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[31] ), .b(\b[30] ), .out0(new_n328));
  and002aa1n02x5               g233(.a(\b[29] ), .b(\a[30] ), .o(new_n329));
  oabi12aa1n02x5               g234(.a(new_n328), .b(\a[30] ), .c(\b[29] ), .out0(new_n330));
  oab012aa1n02x4               g235(.a(new_n330), .b(new_n321), .c(new_n329), .out0(new_n331));
  inv000aa1d42x5               g236(.a(new_n326), .o1(new_n332));
  oao003aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n332), .c(new_n294), .d(new_n291), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n334), .b(new_n328), .c(new_n327), .d(new_n331), .o1(\s[31] ));
  oai012aa1n02x5               g240(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n336));
  xnrb03aa1n02x5               g241(.a(new_n336), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n02x5               g242(.a(new_n107), .b(new_n109), .o1(new_n338));
  nona22aa1n02x4               g243(.a(new_n336), .b(new_n108), .c(new_n101), .out0(new_n339));
  xorc02aa1n02x5               g244(.a(\a[4] ), .b(\b[3] ), .out0(new_n340));
  nona22aa1n02x4               g245(.a(new_n339), .b(new_n340), .c(new_n101), .out0(new_n341));
  oai012aa1n02x5               g246(.a(new_n341), .b(new_n338), .c(new_n102), .o1(\s[4] ));
  xnbna2aa1n03x5               g247(.a(new_n116), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  and003aa1n02x5               g248(.a(new_n107), .b(new_n116), .c(new_n109), .o(new_n344));
  norb02aa1n02x5               g249(.a(new_n113), .b(new_n112), .out0(new_n345));
  aoai13aa1n02x5               g250(.a(new_n345), .b(new_n344), .c(\b[4] ), .d(\a[5] ), .o1(new_n346));
  aboi22aa1n03x5               g251(.a(new_n112), .b(new_n113), .c(\a[5] ), .d(\b[4] ), .out0(new_n347));
  oaib12aa1n02x5               g252(.a(new_n346), .b(new_n344), .c(new_n347), .out0(\s[6] ));
  nanb02aa1n02x5               g253(.a(new_n111), .b(new_n110), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n346), .c(new_n113), .out0(\s[7] ));
  aoai13aa1n02x5               g255(.a(new_n110), .b(new_n111), .c(new_n346), .d(new_n113), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n110), .b(new_n117), .out0(new_n352));
  aoai13aa1n02x5               g257(.a(new_n352), .b(new_n111), .c(new_n346), .d(new_n113), .o1(new_n353));
  aob012aa1n02x5               g258(.a(new_n353), .b(new_n351), .c(new_n117), .out0(\s[8] ));
  nanb02aa1n02x5               g259(.a(new_n118), .b(new_n338), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n125), .b(new_n355), .c(new_n123), .out0(\s[9] ));
endmodule


