// Benchmark "adder" written by ABC on Thu Jul 18 00:10:38 2024

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
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n321, new_n322, new_n324,
    new_n325, new_n328, new_n330, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d28x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n06x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand42aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  norb03aa1n03x5               g008(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand23aa1n02x5               g013(.a(new_n107), .b(new_n102), .c(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oaoi13aa1n12x5               g015(.a(new_n100), .b(new_n110), .c(new_n104), .d(new_n109), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\a[6] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[5] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(new_n113), .b(new_n112), .o1(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[7] ), .b(\b[6] ), .out0(new_n116));
  nor042aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand02aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n04x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nanb03aa1d18x5               g024(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n120));
  nano23aa1n03x7               g025(.a(new_n115), .b(new_n120), .c(new_n116), .d(new_n114), .out0(new_n121));
  aoi112aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  nanb03aa1n03x5               g028(.a(new_n120), .b(new_n123), .c(new_n116), .out0(new_n124));
  nona22aa1n06x5               g029(.a(new_n124), .b(new_n122), .c(new_n117), .out0(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n121), .d(new_n111), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n20x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n06x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(new_n100), .o1(new_n132));
  oaih12aa1n02x5               g037(.a(new_n110), .b(new_n104), .c(new_n109), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  nona23aa1n09x5               g039(.a(new_n116), .b(new_n114), .c(new_n120), .d(new_n115), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n120), .o1(new_n136));
  aoi113aa1n02x5               g041(.a(new_n117), .b(new_n122), .c(new_n136), .d(new_n116), .e(new_n123), .o1(new_n137));
  tech160nm_fioai012aa1n03p5x5 g042(.a(new_n137), .b(new_n134), .c(new_n135), .o1(new_n138));
  oaoi03aa1n02x5               g043(.a(new_n97), .b(new_n98), .c(new_n138), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n130), .o1(new_n140));
  aoai13aa1n12x5               g045(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n141));
  nor002aa1n12x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  oaoi13aa1n02x5               g050(.a(new_n145), .b(new_n141), .c(new_n139), .d(new_n140), .o1(new_n146));
  tech160nm_fiaoi012aa1n05x5   g051(.a(new_n140), .b(new_n127), .c(new_n99), .o1(new_n147));
  nano22aa1n02x4               g052(.a(new_n147), .b(new_n141), .c(new_n145), .out0(new_n148));
  norp02aa1n02x5               g053(.a(new_n146), .b(new_n148), .o1(\s[11] ));
  inv000aa1d42x5               g054(.a(new_n142), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n141), .o1(new_n151));
  oai012aa1n02x5               g056(.a(new_n144), .b(new_n147), .c(new_n151), .o1(new_n152));
  nor042aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  tech160nm_fiaoi012aa1n02p5x5 g060(.a(new_n155), .b(new_n152), .c(new_n150), .o1(new_n156));
  nano22aa1n02x4               g061(.a(new_n146), .b(new_n150), .c(new_n155), .out0(new_n157));
  norp02aa1n02x5               g062(.a(new_n156), .b(new_n157), .o1(\s[12] ));
  nona23aa1n06x5               g063(.a(new_n154), .b(new_n143), .c(new_n142), .d(new_n153), .out0(new_n159));
  nano22aa1n02x4               g064(.a(new_n159), .b(new_n126), .c(new_n130), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n125), .c(new_n121), .d(new_n111), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n142), .b(new_n154), .o1(new_n162));
  oai122aa1n12x5               g067(.a(new_n162), .b(new_n159), .c(new_n141), .d(\b[11] ), .e(\a[12] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  tech160nm_fixorc02aa1n02p5x5 g069(.a(\a[13] ), .b(\b[12] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n161), .c(new_n164), .out0(\s[13] ));
  inv000aa1d42x5               g071(.a(\a[13] ), .o1(new_n167));
  inv000aa1d42x5               g072(.a(\b[12] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n168), .b(new_n167), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n165), .b(new_n163), .c(new_n138), .d(new_n160), .o1(new_n170));
  nor022aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand02aa1n06x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nanb02aa1d24x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n170), .c(new_n169), .out0(\s[14] ));
  nanp02aa1n02x5               g080(.a(\b[12] ), .b(\a[13] ), .o1(new_n176));
  nano22aa1d15x5               g081(.a(new_n173), .b(new_n169), .c(new_n176), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n172), .b(new_n171), .c(new_n167), .d(new_n168), .o1(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n178), .c(new_n161), .d(new_n164), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  xnrc02aa1n12x5               g087(.a(\b[14] ), .b(\a[15] ), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  xorc02aa1n02x5               g089(.a(\a[16] ), .b(\b[15] ), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n182), .c(new_n180), .d(new_n184), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n182), .b(new_n185), .c(new_n180), .d(new_n184), .o1(new_n187));
  norb02aa1n03x4               g092(.a(new_n186), .b(new_n187), .out0(\s[16] ));
  nano23aa1n02x4               g093(.a(new_n142), .b(new_n153), .c(new_n154), .d(new_n143), .out0(new_n189));
  xnrc02aa1n02x5               g094(.a(\b[15] ), .b(\a[16] ), .out0(new_n190));
  nona22aa1n03x5               g095(.a(new_n177), .b(new_n183), .c(new_n190), .out0(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n189), .c(new_n130), .d(new_n126), .out0(new_n192));
  aoai13aa1n12x5               g097(.a(new_n192), .b(new_n125), .c(new_n121), .d(new_n111), .o1(new_n193));
  nano32aa1n02x4               g098(.a(new_n183), .b(new_n174), .c(new_n165), .d(new_n185), .out0(new_n194));
  aoi112aa1n02x5               g099(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n195));
  norp03aa1n02x5               g100(.a(new_n190), .b(new_n183), .c(new_n179), .o1(new_n196));
  oabi12aa1n02x5               g101(.a(new_n196), .b(\a[16] ), .c(\b[15] ), .out0(new_n197));
  aoi112aa1n09x5               g102(.a(new_n197), .b(new_n195), .c(new_n163), .d(new_n194), .o1(new_n198));
  xorc02aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n198), .c(new_n193), .out0(\s[17] ));
  inv000aa1d42x5               g105(.a(\a[17] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\b[16] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  norp02aa1n02x5               g108(.a(\b[15] ), .b(\a[16] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n163), .b(new_n194), .o1(new_n205));
  nona32aa1n03x5               g110(.a(new_n205), .b(new_n196), .c(new_n195), .d(new_n204), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n199), .b(new_n206), .c(new_n138), .d(new_n192), .o1(new_n207));
  norp02aa1n24x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand42aa1n03x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nanb02aa1n06x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  xobna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n203), .out0(\s[18] ));
  nanp02aa1n02x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  nano22aa1n09x5               g117(.a(new_n210), .b(new_n203), .c(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n09x5               g119(.a(new_n209), .b(new_n208), .c(new_n201), .d(new_n202), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n214), .c(new_n198), .d(new_n193), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand42aa1n02x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nor042aa1n02x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n219), .c(new_n216), .d(new_n220), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(new_n219), .b(new_n223), .c(new_n216), .d(new_n220), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n224), .b(new_n225), .out0(\s[20] ));
  nano23aa1n06x5               g131(.a(new_n219), .b(new_n221), .c(new_n222), .d(new_n220), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n213), .b(new_n227), .o1(new_n228));
  nona23aa1n09x5               g133(.a(new_n222), .b(new_n220), .c(new_n219), .d(new_n221), .out0(new_n229));
  tech160nm_fiaoi012aa1n05x5   g134(.a(new_n221), .b(new_n219), .c(new_n222), .o1(new_n230));
  oai012aa1n12x5               g135(.a(new_n230), .b(new_n229), .c(new_n215), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n228), .c(new_n198), .d(new_n193), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[21] ), .b(\b[20] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoi112aa1n02x5               g143(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n03x4               g144(.a(new_n238), .b(new_n239), .out0(\s[22] ));
  inv000aa1d42x5               g145(.a(\a[21] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[22] ), .o1(new_n242));
  xroi22aa1d04x5               g147(.a(new_n241), .b(\b[20] ), .c(new_n242), .d(\b[21] ), .out0(new_n243));
  nanp03aa1n02x5               g148(.a(new_n243), .b(new_n213), .c(new_n227), .o1(new_n244));
  inv000aa1n02x5               g149(.a(new_n215), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n230), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n243), .b(new_n246), .c(new_n227), .d(new_n245), .o1(new_n247));
  inv000aa1d42x5               g152(.a(\b[21] ), .o1(new_n248));
  oaoi03aa1n09x5               g153(.a(new_n242), .b(new_n248), .c(new_n235), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(new_n247), .b(new_n249), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n244), .c(new_n198), .d(new_n193), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  xorc02aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n257));
  aoi112aa1n02x7               g162(.a(new_n254), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n258));
  norb02aa1n03x4               g163(.a(new_n257), .b(new_n258), .out0(\s[24] ));
  and002aa1n02x5               g164(.a(new_n256), .b(new_n255), .o(new_n260));
  nanb03aa1n02x5               g165(.a(new_n228), .b(new_n260), .c(new_n243), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n260), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n254), .o1(new_n263));
  oao003aa1n02x5               g168(.a(\a[24] ), .b(\b[23] ), .c(new_n263), .carry(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n262), .c(new_n247), .d(new_n249), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n261), .c(new_n198), .d(new_n193), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xorc02aa1n06x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n272));
  aoi112aa1n02x5               g177(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n270), .o1(new_n273));
  norb02aa1n03x4               g178(.a(new_n272), .b(new_n273), .out0(\s[26] ));
  and002aa1n02x7               g179(.a(new_n271), .b(new_n270), .o(new_n275));
  nano22aa1n03x7               g180(.a(new_n244), .b(new_n260), .c(new_n275), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n206), .c(new_n138), .d(new_n192), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n269), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .c(new_n278), .carry(new_n279));
  aobi12aa1n09x5               g184(.a(new_n279), .b(new_n265), .c(new_n275), .out0(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n277), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  nand02aa1d06x5               g189(.a(new_n198), .b(new_n193), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n249), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n260), .b(new_n286), .c(new_n231), .d(new_n243), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n275), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n279), .b(new_n288), .c(new_n287), .d(new_n264), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n281), .b(new_n289), .c(new_n285), .d(new_n276), .o1(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[27] ), .b(\a[28] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n291), .b(new_n290), .c(new_n284), .o1(new_n292));
  aobi12aa1n02x5               g197(.a(new_n281), .b(new_n277), .c(new_n280), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n284), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[28] ));
  norb02aa1n02x5               g200(.a(new_n281), .b(new_n291), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n289), .c(new_n285), .d(new_n276), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x5               g205(.a(new_n296), .b(new_n277), .c(new_n280), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g209(.a(new_n281), .b(new_n299), .c(new_n291), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n289), .c(new_n285), .d(new_n276), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[29] ), .b(\a[30] ), .out0(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  aobi12aa1n02x5               g214(.a(new_n305), .b(new_n277), .c(new_n280), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n310), .b(new_n307), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[30] ));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n305), .b(new_n308), .out0(new_n314));
  aobi12aa1n02x7               g219(.a(new_n314), .b(new_n277), .c(new_n280), .out0(new_n315));
  oao003aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n307), .carry(new_n316));
  nano22aa1n02x4               g221(.a(new_n315), .b(new_n313), .c(new_n316), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n314), .b(new_n289), .c(new_n285), .d(new_n276), .o1(new_n318));
  aoi012aa1n03x5               g223(.a(new_n313), .b(new_n318), .c(new_n316), .o1(new_n319));
  nor002aa1n02x5               g224(.a(new_n319), .b(new_n317), .o1(\s[31] ));
  norp02aa1n02x5               g225(.a(new_n104), .b(new_n109), .o1(new_n321));
  aboi22aa1n03x5               g226(.a(new_n104), .b(new_n102), .c(new_n108), .d(new_n107), .out0(new_n322));
  norp02aa1n02x5               g227(.a(new_n322), .b(new_n321), .o1(\s[3] ));
  xnrc02aa1n02x5               g228(.a(\b[3] ), .b(\a[4] ), .out0(new_n324));
  nano22aa1n02x4               g229(.a(new_n321), .b(new_n107), .c(new_n324), .out0(new_n325));
  oaoi13aa1n02x5               g230(.a(new_n325), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g231(.a(new_n115), .b(new_n133), .c(new_n132), .out0(\s[5] ));
  oaoi03aa1n02x5               g232(.a(\a[5] ), .b(\b[4] ), .c(new_n134), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g234(.a(new_n112), .b(new_n113), .c(new_n328), .o1(new_n330));
  xnrc02aa1n02x5               g235(.a(new_n330), .b(new_n116), .out0(\s[7] ));
  oaoi03aa1n02x5               g236(.a(\a[7] ), .b(\b[6] ), .c(new_n330), .o1(new_n332));
  xorb03aa1n02x5               g237(.a(new_n332), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g238(.a(new_n138), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


