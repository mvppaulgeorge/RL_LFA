// Benchmark "adder" written by ABC on Thu Jul 18 08:55:49 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n332, new_n333, new_n336, new_n338, new_n339, new_n340, new_n341,
    new_n342, new_n343, new_n345, new_n346;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d08x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d08x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(\b[2] ), .b(new_n105), .out0(new_n106));
  oao003aa1n06x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1n03x5               g012(.a(new_n107), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n108));
  nor042aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand02aa1d24x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n09x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n08x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1d15x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  inv000aa1n02x5               g018(.a(new_n113), .o1(new_n114));
  tech160nm_fixorc02aa1n04x5   g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  tech160nm_fixorc02aa1n05x5   g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nano22aa1n03x7               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  orn002aa1n24x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n12x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  ao0012aa1n03x7               g024(.a(new_n109), .b(new_n111), .c(new_n110), .o(new_n120));
  aoi012aa1d24x5               g025(.a(new_n120), .b(new_n113), .c(new_n119), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n121), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n124));
  nor022aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand02aa1n08x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n12x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  xobna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  nand42aa1n16x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nor002aa1n16x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n09x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  nona22aa1n02x4               g036(.a(new_n124), .b(new_n125), .c(new_n97), .out0(new_n132));
  xobna2aa1n03x5               g037(.a(new_n131), .b(new_n132), .c(new_n126), .out0(\s[11] ));
  inv000aa1d42x5               g038(.a(new_n130), .o1(new_n134));
  xorc02aa1n02x5               g039(.a(\a[4] ), .b(\b[3] ), .out0(new_n135));
  xorc02aa1n02x5               g040(.a(\a[3] ), .b(\b[2] ), .out0(new_n136));
  nanb03aa1n06x5               g041(.a(new_n102), .b(new_n136), .c(new_n135), .out0(new_n137));
  nand23aa1n03x5               g042(.a(new_n113), .b(new_n115), .c(new_n116), .o1(new_n138));
  aoai13aa1n12x5               g043(.a(new_n121), .b(new_n138), .c(new_n137), .d(new_n107), .o1(new_n139));
  oai022aa1d24x5               g044(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n140));
  nano22aa1n02x4               g045(.a(new_n130), .b(new_n126), .c(new_n129), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n140), .c(new_n139), .d(new_n123), .o1(new_n142));
  orn002aa1n03x5               g047(.a(\a[12] ), .b(\b[11] ), .o(new_n143));
  nand42aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1n02x5               g049(.a(new_n143), .b(new_n144), .o1(new_n145));
  xobna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n134), .out0(\s[12] ));
  nona23aa1n09x5               g051(.a(new_n131), .b(new_n123), .c(new_n145), .d(new_n127), .out0(new_n147));
  nanb02aa1n06x5               g052(.a(new_n147), .b(new_n139), .out0(new_n148));
  aoai13aa1n06x5               g053(.a(new_n129), .b(new_n130), .c(new_n140), .d(new_n126), .o1(new_n149));
  oaoi03aa1n12x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n149), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nor022aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  tech160nm_finand02aa1n03p5x5 g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(\a[14] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[12] ), .o1(new_n158));
  nanp02aa1n06x5               g063(.a(new_n148), .b(new_n151), .o1(new_n159));
  oaoi03aa1n03x5               g064(.a(new_n157), .b(new_n158), .c(new_n159), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(new_n156), .out0(\s[14] ));
  nor002aa1d24x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand02aa1d12x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1d18x5               g068(.a(new_n163), .b(new_n153), .c(new_n152), .d(new_n162), .out0(new_n164));
  nanb02aa1n02x5               g069(.a(new_n164), .b(new_n159), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n163), .b(new_n162), .c(new_n157), .d(new_n158), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n164), .c(new_n148), .d(new_n151), .o1(new_n167));
  nor022aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand02aa1d24x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n06x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  oaoi13aa1n02x5               g075(.a(new_n170), .b(new_n163), .c(new_n152), .d(new_n162), .o1(new_n171));
  aoi022aa1n02x5               g076(.a(new_n171), .b(new_n165), .c(new_n167), .d(new_n170), .o1(\s[15] ));
  nor002aa1n20x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1d28x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1d21x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n168), .c(new_n167), .d(new_n169), .o1(new_n177));
  nand42aa1n02x5               g082(.a(new_n167), .b(new_n170), .o1(new_n178));
  nona22aa1n03x5               g083(.a(new_n178), .b(new_n176), .c(new_n168), .out0(new_n179));
  nanp02aa1n03x5               g084(.a(new_n179), .b(new_n177), .o1(\s[16] ));
  nano22aa1n12x5               g085(.a(new_n164), .b(new_n170), .c(new_n175), .out0(new_n181));
  norb02aa1n12x5               g086(.a(new_n181), .b(new_n147), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n183));
  inv030aa1n02x5               g088(.a(new_n168), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n169), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n173), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n185), .c(new_n166), .d(new_n184), .o1(new_n187));
  aoi022aa1d24x5               g092(.a(new_n150), .b(new_n181), .c(new_n174), .d(new_n187), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n183), .c(new_n188), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[16] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n129), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[9] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[8] ), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n126), .b(new_n125), .c(new_n195), .d(new_n196), .o1(new_n197));
  aoai13aa1n02x7               g102(.a(new_n143), .b(new_n194), .c(new_n197), .d(new_n134), .o1(new_n198));
  nand03aa1n02x5               g103(.a(new_n198), .b(new_n181), .c(new_n144), .o1(new_n199));
  nand02aa1n04x5               g104(.a(new_n187), .b(new_n174), .o1(new_n200));
  nand02aa1d06x5               g105(.a(new_n199), .b(new_n200), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n189), .b(new_n201), .c(new_n139), .d(new_n182), .o1(new_n202));
  nor002aa1n03x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nand42aa1n03x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nanb02aa1n09x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  xobna2aa1n03x5               g110(.a(new_n205), .b(new_n202), .c(new_n193), .out0(\s[18] ));
  norb02aa1n02x5               g111(.a(new_n189), .b(new_n205), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n201), .c(new_n139), .d(new_n182), .o1(new_n208));
  aoai13aa1n09x5               g113(.a(new_n204), .b(new_n203), .c(new_n191), .d(new_n192), .o1(new_n209));
  nor022aa1n16x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n208), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g119(.a(new_n208), .b(new_n209), .o1(new_n215));
  nor002aa1d32x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand22aa1n09x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n210), .c(new_n215), .d(new_n212), .o1(new_n219));
  nand02aa1d06x5               g124(.a(new_n183), .b(new_n188), .o1(new_n220));
  oaoi03aa1n02x5               g125(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n212), .b(new_n221), .c(new_n220), .d(new_n207), .o1(new_n222));
  nona22aa1n02x5               g127(.a(new_n222), .b(new_n218), .c(new_n210), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n219), .b(new_n223), .o1(\s[20] ));
  nona23aa1d18x5               g129(.a(new_n217), .b(new_n211), .c(new_n210), .d(new_n216), .out0(new_n225));
  norb03aa1n02x5               g130(.a(new_n189), .b(new_n225), .c(new_n205), .out0(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  ao0012aa1n12x5               g132(.a(new_n216), .b(new_n210), .c(new_n217), .o(new_n228));
  oabi12aa1n18x5               g133(.a(new_n228), .b(new_n225), .c(new_n209), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n04x5               g135(.a(new_n230), .b(new_n227), .c(new_n183), .d(new_n188), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n233), .b(new_n229), .c(new_n220), .d(new_n226), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n231), .c(new_n233), .o1(\s[21] ));
  nor002aa1n02x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  tech160nm_fixnrc02aa1n05x5   g141(.a(\b[21] ), .b(\a[22] ), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n236), .c(new_n231), .d(new_n233), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(new_n231), .b(new_n233), .o1(new_n239));
  nona22aa1n02x4               g144(.a(new_n239), .b(new_n237), .c(new_n236), .out0(new_n240));
  nanp02aa1n03x5               g145(.a(new_n240), .b(new_n238), .o1(\s[22] ));
  nor042aa1n09x5               g146(.a(new_n237), .b(new_n232), .o1(new_n242));
  nona23aa1d16x5               g147(.a(new_n242), .b(new_n189), .c(new_n225), .d(new_n205), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n201), .c(new_n139), .d(new_n182), .o1(new_n245));
  nano23aa1n09x5               g150(.a(new_n210), .b(new_n216), .c(new_n217), .d(new_n211), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n242), .b(new_n228), .c(new_n246), .d(new_n221), .o1(new_n247));
  inv000aa1d42x5               g152(.a(\a[22] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\b[21] ), .o1(new_n249));
  oao003aa1n02x5               g154(.a(new_n248), .b(new_n249), .c(new_n236), .carry(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n247), .b(new_n251), .o1(new_n252));
  nanb02aa1n03x5               g157(.a(new_n252), .b(new_n245), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n229), .d(new_n242), .o1(new_n255));
  aoi022aa1n02x5               g160(.a(new_n253), .b(new_n254), .c(new_n245), .d(new_n255), .o1(\s[23] ));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  tech160nm_fixnrc02aa1n05x5   g162(.a(\b[23] ), .b(\a[24] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n257), .c(new_n253), .d(new_n254), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n254), .b(new_n252), .c(new_n220), .d(new_n244), .o1(new_n260));
  nona22aa1n02x5               g165(.a(new_n260), .b(new_n258), .c(new_n257), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n259), .b(new_n261), .o1(\s[24] ));
  norb02aa1n03x5               g167(.a(new_n254), .b(new_n258), .out0(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  nor042aa1n03x5               g169(.a(new_n243), .b(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n201), .c(new_n139), .d(new_n182), .o1(new_n266));
  orn002aa1n02x5               g171(.a(\a[23] ), .b(\b[22] ), .o(new_n267));
  oao003aa1n02x5               g172(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .carry(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n264), .c(new_n247), .d(new_n251), .o1(new_n269));
  nanb02aa1n03x5               g174(.a(new_n269), .b(new_n266), .out0(new_n270));
  xorb03aa1n02x5               g175(.a(new_n270), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  xorc02aa1n03x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  xorc02aa1n12x5               g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n272), .c(new_n270), .d(new_n273), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n273), .b(new_n269), .c(new_n220), .d(new_n265), .o1(new_n277));
  nona22aa1n02x5               g182(.a(new_n277), .b(new_n275), .c(new_n272), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n276), .b(new_n278), .o1(\s[26] ));
  and002aa1n09x5               g184(.a(new_n274), .b(new_n273), .o(new_n280));
  nano22aa1n12x5               g185(.a(new_n243), .b(new_n280), .c(new_n263), .out0(new_n281));
  aoai13aa1n09x5               g186(.a(new_n281), .b(new_n201), .c(new_n139), .d(new_n182), .o1(new_n282));
  nand42aa1n02x5               g187(.a(new_n269), .b(new_n280), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n284));
  oab012aa1n02x4               g189(.a(new_n284), .b(\a[26] ), .c(\b[25] ), .out0(new_n285));
  nand23aa1n06x5               g190(.a(new_n282), .b(new_n283), .c(new_n285), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n283), .c(new_n285), .out0(new_n288));
  aoi022aa1n02x5               g193(.a(new_n288), .b(new_n282), .c(new_n286), .d(new_n287), .o1(\s[27] ));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[27] ), .b(\a[28] ), .o1(new_n291));
  nand42aa1n03x5               g196(.a(\b[27] ), .b(\a[28] ), .o1(new_n292));
  nanb02aa1n02x5               g197(.a(new_n291), .b(new_n292), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n286), .d(new_n287), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n281), .o1(new_n295));
  aoi012aa1n06x5               g200(.a(new_n295), .b(new_n183), .c(new_n188), .o1(new_n296));
  aoai13aa1n04x5               g201(.a(new_n263), .b(new_n250), .c(new_n229), .d(new_n242), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n280), .o1(new_n298));
  aoai13aa1n06x5               g203(.a(new_n285), .b(new_n298), .c(new_n297), .d(new_n268), .o1(new_n299));
  oaih12aa1n02x5               g204(.a(new_n287), .b(new_n299), .c(new_n296), .o1(new_n300));
  nona22aa1n02x4               g205(.a(new_n300), .b(new_n293), .c(new_n290), .out0(new_n301));
  nanp02aa1n03x5               g206(.a(new_n294), .b(new_n301), .o1(\s[28] ));
  norb02aa1n12x5               g207(.a(new_n287), .b(new_n293), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n299), .c(new_n296), .o1(new_n304));
  aobi12aa1n06x5               g209(.a(new_n285), .b(new_n269), .c(new_n280), .out0(new_n305));
  inv000aa1d42x5               g210(.a(new_n303), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n292), .b(new_n291), .c(new_n290), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n305), .d(new_n282), .o1(new_n308));
  norp02aa1n02x5               g213(.a(\b[28] ), .b(\a[29] ), .o1(new_n309));
  nand42aa1n03x5               g214(.a(\b[28] ), .b(\a[29] ), .o1(new_n310));
  norb02aa1n02x5               g215(.a(new_n310), .b(new_n309), .out0(new_n311));
  oai022aa1n02x5               g216(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n312));
  aboi22aa1n03x5               g217(.a(new_n309), .b(new_n310), .c(new_n312), .d(new_n292), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n308), .b(new_n311), .c(new_n304), .d(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g220(.a(new_n293), .b(new_n287), .c(new_n311), .out0(new_n316));
  oaih12aa1n02x5               g221(.a(new_n316), .b(new_n299), .c(new_n296), .o1(new_n317));
  inv000aa1n02x5               g222(.a(new_n316), .o1(new_n318));
  aoi013aa1n02x4               g223(.a(new_n309), .b(new_n312), .c(new_n292), .d(new_n310), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n305), .d(new_n282), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .out0(new_n321));
  aoi113aa1n02x5               g226(.a(new_n321), .b(new_n309), .c(new_n312), .d(new_n310), .e(new_n292), .o1(new_n322));
  aoi022aa1n03x5               g227(.a(new_n320), .b(new_n321), .c(new_n317), .d(new_n322), .o1(\s[30] ));
  nanp03aa1n02x5               g228(.a(new_n303), .b(new_n311), .c(new_n321), .o1(new_n324));
  oabi12aa1n02x5               g229(.a(new_n324), .b(new_n299), .c(new_n296), .out0(new_n325));
  xorc02aa1n02x5               g230(.a(\a[31] ), .b(\b[30] ), .out0(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n319), .carry(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n327), .b(new_n324), .c(new_n305), .d(new_n282), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n329), .b(new_n326), .c(new_n325), .d(new_n328), .o1(\s[31] ));
  xorb03aa1n02x5               g235(.a(new_n102), .b(\b[2] ), .c(new_n105), .out0(\s[3] ));
  orn002aa1n02x5               g236(.a(\a[4] ), .b(\b[3] ), .o(new_n332));
  oai112aa1n02x5               g237(.a(new_n106), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n333));
  aobi12aa1n02x5               g238(.a(new_n333), .b(new_n108), .c(new_n332), .out0(\s[4] ));
  xnbna2aa1n03x5               g239(.a(new_n116), .b(new_n137), .c(new_n107), .out0(\s[5] ));
  nanp02aa1n02x5               g240(.a(new_n108), .b(new_n116), .o1(new_n336));
  xnbna2aa1n03x5               g241(.a(new_n115), .b(new_n336), .c(new_n118), .out0(\s[6] ));
  inv000aa1d42x5               g242(.a(\a[6] ), .o1(new_n338));
  inv000aa1d42x5               g243(.a(\b[5] ), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n112), .b(new_n111), .out0(new_n340));
  aobi12aa1n06x5               g245(.a(new_n115), .b(new_n336), .c(new_n118), .out0(new_n341));
  aoai13aa1n06x5               g246(.a(new_n340), .b(new_n341), .c(new_n339), .d(new_n338), .o1(new_n342));
  aoi112aa1n02x5               g247(.a(new_n341), .b(new_n340), .c(new_n338), .d(new_n339), .o1(new_n343));
  norb02aa1n02x5               g248(.a(new_n342), .b(new_n343), .out0(\s[7] ));
  norb02aa1n02x5               g249(.a(new_n110), .b(new_n109), .out0(new_n345));
  orn002aa1n02x5               g250(.a(\a[7] ), .b(\b[6] ), .o(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n345), .b(new_n342), .c(new_n346), .out0(\s[8] ));
  xorb03aa1n02x5               g252(.a(new_n139), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


