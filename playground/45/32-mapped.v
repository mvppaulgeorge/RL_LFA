// Benchmark "adder" written by ABC on Thu Jul 18 11:20:07 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n332,
    new_n334, new_n336, new_n338, new_n340;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  aoi012aa1n12x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand22aa1n04x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  norp02aa1n24x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  tech160nm_finand02aa1n03p5x5 g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  tech160nm_fioai012aa1n04x5   g015(.a(new_n107), .b(new_n108), .c(new_n106), .o1(new_n111));
  oai012aa1n12x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\a[7] ), .o1(new_n113));
  nanb02aa1d36x5               g018(.a(\b[6] ), .b(new_n113), .out0(new_n114));
  nand42aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand42aa1n08x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  norb02aa1n03x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  nand22aa1n09x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nor042aa1n04x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n121), .o1(new_n123));
  and002aa1n12x5               g028(.a(\b[7] ), .b(\a[8] ), .o(new_n124));
  nona23aa1n02x5               g029(.a(new_n123), .b(new_n119), .c(new_n124), .d(new_n120), .out0(new_n125));
  nano32aa1n03x7               g030(.a(new_n125), .b(new_n118), .c(new_n115), .d(new_n114), .out0(new_n126));
  aoai13aa1n02x7               g031(.a(new_n115), .b(new_n120), .c(new_n116), .d(new_n119), .o1(new_n127));
  aoai13aa1n06x5               g032(.a(new_n123), .b(new_n124), .c(new_n127), .d(new_n114), .o1(new_n128));
  nand42aa1n08x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n100), .out0(new_n130));
  aoai13aa1n03x5               g035(.a(new_n130), .b(new_n128), .c(new_n126), .d(new_n112), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n99), .b(new_n131), .c(new_n101), .out0(\s[10] ));
  xnrc02aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .out0(new_n133));
  nona22aa1n02x4               g038(.a(new_n131), .b(new_n100), .c(new_n97), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n133), .b(new_n134), .c(new_n98), .out0(\s[11] ));
  inv000aa1d42x5               g040(.a(\a[12] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(\b[10] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n98), .b(new_n133), .out0(new_n138));
  aboi22aa1n02x7               g043(.a(\a[11] ), .b(new_n137), .c(new_n134), .d(new_n138), .out0(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[11] ), .c(new_n136), .out0(\s[12] ));
  tech160nm_fixnrc02aa1n02p5x5 g045(.a(\b[11] ), .b(\a[12] ), .out0(new_n141));
  nano23aa1n06x5               g046(.a(new_n97), .b(new_n100), .c(new_n129), .d(new_n98), .out0(new_n142));
  nona22aa1d18x5               g047(.a(new_n142), .b(new_n141), .c(new_n133), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n128), .c(new_n126), .d(new_n112), .o1(new_n145));
  inv000aa1d42x5               g050(.a(\b[11] ), .o1(new_n146));
  nand02aa1d10x5               g051(.a(new_n100), .b(new_n98), .o1(new_n147));
  oai122aa1n12x5               g052(.a(new_n147), .b(\b[9] ), .c(\a[10] ), .d(\b[10] ), .e(\a[11] ), .o1(new_n148));
  aoi022aa1d24x5               g053(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n149));
  aoi022aa1d24x5               g054(.a(new_n148), .b(new_n149), .c(new_n146), .d(new_n136), .o1(new_n150));
  nor042aa1n04x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nand42aa1d28x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n145), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g059(.a(\a[13] ), .o1(new_n155));
  inv000aa1d42x5               g060(.a(\b[12] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(new_n145), .b(new_n150), .o1(new_n157));
  oaoi03aa1n03x5               g062(.a(new_n155), .b(new_n156), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv020aa1n02x5               g064(.a(new_n105), .o1(new_n160));
  nano23aa1d15x5               g065(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n161));
  aobi12aa1n06x5               g066(.a(new_n111), .b(new_n161), .c(new_n160), .out0(new_n162));
  nano32aa1n02x4               g067(.a(new_n116), .b(new_n114), .c(new_n117), .d(new_n115), .out0(new_n163));
  nano23aa1n02x4               g068(.a(new_n120), .b(new_n124), .c(new_n123), .d(new_n119), .out0(new_n164));
  nanp02aa1n02x5               g069(.a(new_n164), .b(new_n163), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(new_n127), .b(new_n114), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n121), .b(new_n122), .c(new_n166), .o1(new_n167));
  oai012aa1n06x5               g072(.a(new_n167), .b(new_n162), .c(new_n165), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n150), .o1(new_n169));
  nor042aa1n09x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n16x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nano23aa1d15x5               g076(.a(new_n151), .b(new_n170), .c(new_n171), .d(new_n152), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n169), .c(new_n168), .d(new_n144), .o1(new_n173));
  aoai13aa1n06x5               g078(.a(new_n171), .b(new_n170), .c(new_n155), .d(new_n156), .o1(new_n174));
  nor042aa1n04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand02aa1d16x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n173), .c(new_n174), .out0(\s[15] ));
  inv000aa1d42x5               g083(.a(new_n172), .o1(new_n179));
  aoai13aa1n04x5               g084(.a(new_n174), .b(new_n179), .c(new_n145), .d(new_n150), .o1(new_n180));
  nor042aa1n06x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanp02aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanb02aa1n02x5               g087(.a(new_n181), .b(new_n182), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n175), .c(new_n180), .d(new_n176), .o1(new_n184));
  aoi112aa1n03x5               g089(.a(new_n175), .b(new_n183), .c(new_n180), .d(new_n176), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(\s[16] ));
  nano23aa1d12x5               g091(.a(new_n175), .b(new_n181), .c(new_n182), .d(new_n176), .out0(new_n187));
  nand22aa1n12x5               g092(.a(new_n187), .b(new_n172), .o1(new_n188));
  nor042aa1n09x5               g093(.a(new_n143), .b(new_n188), .o1(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n128), .c(new_n112), .d(new_n126), .o1(new_n190));
  inv000aa1n02x5               g095(.a(new_n175), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n176), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n181), .o1(new_n193));
  aoai13aa1n04x5               g098(.a(new_n193), .b(new_n192), .c(new_n174), .d(new_n191), .o1(new_n194));
  nand42aa1n02x5               g099(.a(new_n194), .b(new_n182), .o1(new_n195));
  tech160nm_fioai012aa1n05x5   g100(.a(new_n195), .b(new_n150), .c(new_n188), .o1(new_n196));
  inv040aa1n08x5               g101(.a(new_n196), .o1(new_n197));
  nanp02aa1n09x5               g102(.a(new_n197), .b(new_n190), .o1(new_n198));
  nor042aa1n09x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  nand42aa1n10x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  norp02aa1n02x5               g106(.a(new_n150), .b(new_n188), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n202), .b(new_n201), .c(new_n182), .d(new_n194), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(new_n198), .b(new_n201), .c(new_n203), .d(new_n190), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g109(.a(new_n199), .b(new_n198), .c(new_n201), .o1(new_n205));
  xnrb03aa1n03x5               g110(.a(new_n205), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n06x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nand42aa1n16x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nano23aa1d15x5               g113(.a(new_n199), .b(new_n207), .c(new_n208), .d(new_n200), .out0(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n196), .c(new_n168), .d(new_n189), .o1(new_n210));
  oa0012aa1n02x5               g115(.a(new_n208), .b(new_n207), .c(new_n199), .o(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nor042aa1n06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand42aa1n06x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n06x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n210), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g122(.a(new_n210), .b(new_n212), .o1(new_n218));
  nor042aa1n06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand22aa1n09x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nanb02aa1n06x5               g125(.a(new_n219), .b(new_n220), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n213), .c(new_n218), .d(new_n214), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n215), .b(new_n211), .c(new_n198), .d(new_n209), .o1(new_n223));
  nona22aa1n02x5               g128(.a(new_n223), .b(new_n221), .c(new_n213), .out0(new_n224));
  nanp02aa1n03x5               g129(.a(new_n222), .b(new_n224), .o1(\s[20] ));
  nanb03aa1d18x5               g130(.a(new_n221), .b(new_n209), .c(new_n215), .out0(new_n226));
  nanb03aa1n06x5               g131(.a(new_n219), .b(new_n220), .c(new_n214), .out0(new_n227));
  orn002aa1n02x5               g132(.a(\a[19] ), .b(\b[18] ), .o(new_n228));
  oai112aa1n06x5               g133(.a(new_n228), .b(new_n208), .c(new_n207), .d(new_n199), .o1(new_n229));
  aoi012aa1d18x5               g134(.a(new_n219), .b(new_n213), .c(new_n220), .o1(new_n230));
  oai012aa1n18x5               g135(.a(new_n230), .b(new_n229), .c(new_n227), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n04x5               g137(.a(new_n232), .b(new_n226), .c(new_n197), .d(new_n190), .o1(new_n233));
  nor042aa1n06x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n226), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n236), .b(new_n231), .c(new_n198), .d(new_n237), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n233), .c(new_n236), .o1(\s[21] ));
  nor042aa1n03x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n234), .c(new_n233), .d(new_n236), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(new_n233), .b(new_n236), .o1(new_n244));
  nona22aa1n02x4               g149(.a(new_n244), .b(new_n242), .c(new_n234), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n245), .b(new_n243), .o1(\s[22] ));
  nano23aa1n06x5               g151(.a(new_n234), .b(new_n240), .c(new_n241), .d(new_n235), .out0(new_n247));
  nanb02aa1n03x5               g152(.a(new_n226), .b(new_n247), .out0(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n197), .c(new_n190), .o1(new_n249));
  oa0012aa1n02x5               g154(.a(new_n241), .b(new_n240), .c(new_n234), .o(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(new_n231), .c(new_n247), .o1(new_n251));
  aoai13aa1n04x5               g156(.a(new_n251), .b(new_n248), .c(new_n197), .d(new_n190), .o1(new_n252));
  xorc02aa1n12x5               g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n253), .b(new_n250), .c(new_n231), .d(new_n247), .o1(new_n254));
  aboi22aa1n03x5               g159(.a(new_n249), .b(new_n254), .c(new_n252), .d(new_n253), .out0(\s[23] ));
  norp02aa1n02x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  tech160nm_fixnrc02aa1n02p5x5 g161(.a(\b[23] ), .b(\a[24] ), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n256), .c(new_n252), .d(new_n253), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n252), .b(new_n253), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n259), .b(new_n257), .c(new_n256), .out0(new_n260));
  nanp02aa1n03x5               g165(.a(new_n260), .b(new_n258), .o1(\s[24] ));
  norb02aa1n03x5               g166(.a(new_n253), .b(new_n257), .out0(new_n262));
  nano22aa1n03x7               g167(.a(new_n226), .b(new_n262), .c(new_n247), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n196), .c(new_n168), .d(new_n189), .o1(new_n264));
  nano22aa1n02x4               g169(.a(new_n219), .b(new_n214), .c(new_n220), .out0(new_n265));
  oai012aa1n02x5               g170(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .o1(new_n266));
  oab012aa1n02x4               g171(.a(new_n266), .b(new_n199), .c(new_n207), .out0(new_n267));
  inv000aa1n02x5               g172(.a(new_n230), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n247), .b(new_n268), .c(new_n267), .d(new_n265), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n250), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n262), .o1(new_n271));
  oai022aa1n02x5               g176(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n272));
  aob012aa1n02x5               g177(.a(new_n272), .b(\b[23] ), .c(\a[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n270), .o1(new_n274));
  nanb02aa1n03x5               g179(.a(new_n274), .b(new_n264), .out0(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  xorc02aa1n06x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  tech160nm_fixnrc02aa1n05x5   g183(.a(\b[25] ), .b(\a[26] ), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n275), .d(new_n278), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n278), .b(new_n274), .c(new_n198), .d(new_n263), .o1(new_n281));
  nona22aa1n02x5               g186(.a(new_n281), .b(new_n279), .c(new_n277), .out0(new_n282));
  nanp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[26] ));
  norb02aa1n02x7               g188(.a(new_n278), .b(new_n279), .out0(new_n284));
  inv000aa1n02x5               g189(.a(new_n284), .o1(new_n285));
  nano23aa1d12x5               g190(.a(new_n285), .b(new_n226), .c(new_n262), .d(new_n247), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n196), .c(new_n168), .d(new_n189), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(\b[25] ), .b(\a[26] ), .o1(new_n288));
  oai022aa1n02x5               g193(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n289));
  aoi022aa1n06x5               g194(.a(new_n274), .b(new_n284), .c(new_n288), .d(new_n289), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n290), .c(new_n287), .out0(\s[27] ));
  nand42aa1n04x5               g197(.a(new_n290), .b(new_n287), .o1(new_n293));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  norp02aa1n02x5               g199(.a(\b[27] ), .b(\a[28] ), .o1(new_n295));
  nand42aa1n03x5               g200(.a(\b[27] ), .b(\a[28] ), .o1(new_n296));
  nanb02aa1n06x5               g201(.a(new_n295), .b(new_n296), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n293), .d(new_n291), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n262), .b(new_n250), .c(new_n231), .d(new_n247), .o1(new_n299));
  nanp02aa1n02x5               g204(.a(new_n289), .b(new_n288), .o1(new_n300));
  aoai13aa1n04x5               g205(.a(new_n300), .b(new_n285), .c(new_n299), .d(new_n273), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n291), .b(new_n301), .c(new_n198), .d(new_n286), .o1(new_n302));
  nona22aa1n02x5               g207(.a(new_n302), .b(new_n297), .c(new_n294), .out0(new_n303));
  nanp02aa1n03x5               g208(.a(new_n298), .b(new_n303), .o1(\s[28] ));
  norb02aa1n06x5               g209(.a(new_n291), .b(new_n297), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n301), .c(new_n198), .d(new_n286), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n295), .b(new_n294), .c(new_n296), .o1(new_n308));
  norb02aa1n02x5               g213(.a(new_n308), .b(new_n307), .out0(new_n309));
  inv000aa1d42x5               g214(.a(new_n305), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n308), .b(new_n310), .c(new_n290), .d(new_n287), .o1(new_n311));
  aoi022aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n306), .d(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g217(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g218(.a(new_n297), .b(new_n291), .c(new_n307), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n301), .c(new_n198), .d(new_n286), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  oao003aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .carry(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n317), .b(new_n319), .c(new_n290), .d(new_n287), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n315), .d(new_n318), .o1(\s[30] ));
  nand03aa1n02x5               g226(.a(new_n305), .b(new_n307), .c(new_n316), .o1(new_n322));
  nanb02aa1n03x5               g227(.a(new_n322), .b(new_n293), .out0(new_n323));
  xorc02aa1n02x5               g228(.a(\a[31] ), .b(\b[30] ), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n325));
  norb02aa1n02x5               g230(.a(new_n325), .b(new_n324), .out0(new_n326));
  aoai13aa1n06x5               g231(.a(new_n325), .b(new_n322), .c(new_n290), .d(new_n287), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n323), .b(new_n326), .c(new_n327), .d(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g233(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g234(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g236(.a(new_n161), .b(new_n160), .o1(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n118), .b(new_n332), .c(new_n111), .out0(\s[5] ));
  oaoi03aa1n02x5               g238(.a(\a[5] ), .b(\b[4] ), .c(new_n162), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioai012aa1n05x5   g240(.a(new_n119), .b(new_n334), .c(new_n120), .o1(new_n336));
  xnbna2aa1n03x5               g241(.a(new_n336), .b(new_n114), .c(new_n115), .out0(\s[7] ));
  oaoi03aa1n02x5               g242(.a(\a[7] ), .b(\b[6] ), .c(new_n336), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g244(.a(new_n128), .b(new_n130), .c(new_n126), .d(new_n112), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n340), .b(new_n168), .c(new_n130), .o1(\s[9] ));
endmodule


