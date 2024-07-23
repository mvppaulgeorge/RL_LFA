// Benchmark "adder" written by ABC on Thu Jul 18 07:21:35 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n331,
    new_n334, new_n336, new_n337, new_n339, new_n340, new_n341, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand02aa1d24x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nor022aa1n16x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  tech160nm_fioai012aa1n03p5x5 g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  and002aa1n12x5               g007(.a(\b[1] ), .b(\a[2] ), .o(new_n103));
  nand02aa1d04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oabi12aa1n02x7               g010(.a(new_n103), .b(new_n104), .c(new_n105), .out0(new_n106));
  nand42aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n02x5               g012(.a(new_n99), .b(new_n107), .c(new_n101), .d(new_n100), .out0(new_n108));
  oai012aa1n06x5               g013(.a(new_n102), .b(new_n108), .c(new_n106), .o1(new_n109));
  nor022aa1n06x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n03x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_fixnrc02aa1n04x5   g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  nor022aa1n16x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand22aa1n12x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n06x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  nor003aa1n03x5               g023(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n119));
  aoi022aa1d18x5               g024(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n120), .o1(new_n121));
  aoi112aa1n03x5               g026(.a(new_n116), .b(new_n110), .c(new_n112), .d(new_n117), .o1(new_n122));
  oai022aa1n03x5               g027(.a(new_n122), .b(new_n121), .c(\b[7] ), .d(\a[8] ), .o1(new_n123));
  nand42aa1n06x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n04x5               g030(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n126));
  nor002aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanp02aa1n09x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  inv020aa1n02x5               g035(.a(new_n129), .o1(new_n131));
  tech160nm_fioai012aa1n05x5   g036(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n132));
  aoai13aa1n06x5               g037(.a(new_n132), .b(new_n131), .c(new_n126), .d(new_n98), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanp02aa1n12x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norp02aa1n12x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n140));
  aoi112aa1n03x5               g045(.a(new_n135), .b(new_n139), .c(new_n133), .d(new_n136), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  oab012aa1n06x5               g047(.a(new_n103), .b(new_n105), .c(new_n104), .out0(new_n143));
  norb03aa1n03x5               g048(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n144));
  nand03aa1n03x5               g049(.a(new_n143), .b(new_n144), .c(new_n107), .o1(new_n145));
  nano23aa1n02x5               g050(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n146));
  nona22aa1n03x5               g051(.a(new_n146), .b(new_n115), .c(new_n118), .out0(new_n147));
  norp02aa1n02x5               g052(.a(\b[7] ), .b(\a[8] ), .o1(new_n148));
  oab012aa1n06x5               g053(.a(new_n148), .b(new_n122), .c(new_n121), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n147), .c(new_n145), .d(new_n102), .o1(new_n150));
  nano23aa1n09x5               g055(.a(new_n135), .b(new_n137), .c(new_n138), .d(new_n136), .out0(new_n151));
  nano23aa1n03x5               g056(.a(new_n97), .b(new_n127), .c(new_n128), .d(new_n124), .out0(new_n152));
  nand02aa1n03x5               g057(.a(new_n152), .b(new_n151), .o1(new_n153));
  nanb02aa1n06x5               g058(.a(new_n153), .b(new_n150), .out0(new_n154));
  inv020aa1n02x5               g059(.a(new_n132), .o1(new_n155));
  oai012aa1n02x5               g060(.a(new_n138), .b(new_n137), .c(new_n135), .o1(new_n156));
  aobi12aa1n06x5               g061(.a(new_n156), .b(new_n151), .c(new_n155), .out0(new_n157));
  nanp02aa1n03x5               g062(.a(new_n154), .b(new_n157), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand42aa1d28x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  aoi012aa1n03x5               g066(.a(new_n160), .b(new_n158), .c(new_n161), .o1(new_n162));
  xnrb03aa1n03x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n12x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1n16x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nano23aa1d15x5               g070(.a(new_n160), .b(new_n164), .c(new_n165), .d(new_n161), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  oai012aa1n02x7               g072(.a(new_n165), .b(new_n164), .c(new_n160), .o1(new_n168));
  aoai13aa1n04x5               g073(.a(new_n168), .b(new_n167), .c(new_n154), .d(new_n157), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand02aa1d10x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanb02aa1n18x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  nor042aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n06x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n171), .c(new_n169), .d(new_n174), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(new_n169), .b(new_n174), .o1(new_n179));
  nona22aa1n02x4               g084(.a(new_n179), .b(new_n177), .c(new_n171), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n178), .o1(\s[16] ));
  nano23aa1n03x7               g086(.a(new_n171), .b(new_n175), .c(new_n176), .d(new_n172), .out0(new_n182));
  nano22aa1n12x5               g087(.a(new_n153), .b(new_n166), .c(new_n182), .out0(new_n183));
  aoai13aa1n09x5               g088(.a(new_n183), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n184));
  nona23aa1n02x4               g089(.a(new_n138), .b(new_n136), .c(new_n135), .d(new_n137), .out0(new_n185));
  tech160nm_fioai012aa1n03p5x5 g090(.a(new_n156), .b(new_n185), .c(new_n132), .o1(new_n186));
  nand02aa1n02x5               g091(.a(new_n182), .b(new_n166), .o1(new_n187));
  oai012aa1n02x5               g092(.a(new_n176), .b(new_n175), .c(new_n171), .o1(new_n188));
  oai013aa1n03x4               g093(.a(new_n188), .b(new_n168), .c(new_n173), .d(new_n177), .o1(new_n189));
  aoib12aa1n09x5               g094(.a(new_n189), .b(new_n186), .c(new_n187), .out0(new_n190));
  nanp02aa1n06x5               g095(.a(new_n184), .b(new_n190), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n03x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  oabi12aa1n06x5               g102(.a(new_n189), .b(new_n157), .c(new_n187), .out0(new_n198));
  xroi22aa1d06x4               g103(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n199));
  aoai13aa1n03x5               g104(.a(new_n199), .b(new_n198), .c(new_n150), .d(new_n183), .o1(new_n200));
  nand22aa1n03x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  tech160nm_fiaoi012aa1n05x5   g107(.a(new_n202), .b(new_n194), .c(new_n195), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n201), .b(new_n203), .out0(new_n204));
  inv000aa1n02x5               g109(.a(new_n204), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\a[19] ), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\b[18] ), .o1(new_n207));
  nanp02aa1n12x5               g112(.a(new_n207), .b(new_n206), .o1(new_n208));
  nand42aa1n08x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanp02aa1n09x5               g114(.a(new_n208), .b(new_n209), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n200), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g118(.a(new_n208), .b(new_n210), .c(new_n200), .d(new_n205), .o1(new_n214));
  nor042aa1n06x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanp02aa1n12x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n06x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  inv040aa1n03x5               g122(.a(new_n217), .o1(new_n218));
  nanp02aa1n03x5               g123(.a(new_n214), .b(new_n218), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n211), .b(new_n204), .c(new_n191), .d(new_n199), .o1(new_n220));
  nanp03aa1n03x5               g125(.a(new_n220), .b(new_n208), .c(new_n217), .o1(new_n221));
  nanp02aa1n03x5               g126(.a(new_n219), .b(new_n221), .o1(\s[20] ));
  nona22aa1n09x5               g127(.a(new_n199), .b(new_n210), .c(new_n218), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n216), .b(new_n215), .c(new_n206), .d(new_n207), .o1(new_n224));
  oaih22aa1n04x5               g129(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n225));
  nand23aa1n04x5               g130(.a(new_n225), .b(new_n208), .c(new_n201), .o1(new_n226));
  nanb03aa1d18x5               g131(.a(new_n215), .b(new_n216), .c(new_n209), .out0(new_n227));
  oai012aa1d24x5               g132(.a(new_n224), .b(new_n226), .c(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n223), .c(new_n184), .d(new_n190), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[20] ), .b(\a[21] ), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[21] ), .b(\a[22] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n232), .c(new_n230), .d(new_n234), .o1(new_n236));
  aoi112aa1n03x4               g141(.a(new_n232), .b(new_n235), .c(new_n230), .d(new_n234), .o1(new_n237));
  nanb02aa1n03x5               g142(.a(new_n237), .b(new_n236), .out0(\s[22] ));
  nor042aa1n06x5               g143(.a(new_n235), .b(new_n233), .o1(new_n239));
  nona23aa1n09x5               g144(.a(new_n199), .b(new_n239), .c(new_n218), .d(new_n210), .out0(new_n240));
  inv000aa1d42x5               g145(.a(\a[22] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\b[21] ), .o1(new_n242));
  oao003aa1n02x5               g147(.a(new_n241), .b(new_n242), .c(new_n232), .carry(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n228), .c(new_n239), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n240), .c(new_n184), .d(new_n190), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  tech160nm_fixorc02aa1n03p5x5 g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  tech160nm_fixnrc02aa1n05x5   g153(.a(\b[23] ), .b(\a[24] ), .out0(new_n249));
  aoai13aa1n03x5               g154(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n250));
  aoi112aa1n03x4               g155(.a(new_n247), .b(new_n249), .c(new_n245), .d(new_n248), .o1(new_n251));
  nanb02aa1n03x5               g156(.a(new_n251), .b(new_n250), .out0(\s[24] ));
  inv000aa1n02x5               g157(.a(new_n239), .o1(new_n253));
  nanb02aa1n09x5               g158(.a(new_n249), .b(new_n248), .out0(new_n254));
  nor043aa1n03x5               g159(.a(new_n223), .b(new_n253), .c(new_n254), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n198), .c(new_n150), .d(new_n183), .o1(new_n256));
  inv030aa1n04x5               g161(.a(new_n224), .o1(new_n257));
  oai012aa1n02x5               g162(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .o1(new_n258));
  nor042aa1n02x5               g163(.a(new_n203), .b(new_n258), .o1(new_n259));
  nano22aa1n02x5               g164(.a(new_n215), .b(new_n209), .c(new_n216), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n239), .b(new_n257), .c(new_n259), .d(new_n260), .o1(new_n261));
  inv020aa1n02x5               g166(.a(new_n243), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\a[24] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(\b[23] ), .o1(new_n264));
  oao003aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n247), .carry(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n254), .c(new_n261), .d(new_n262), .o1(new_n267));
  inv040aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n256), .c(new_n268), .out0(\s[25] ));
  nand42aa1n03x5               g175(.a(new_n256), .b(new_n268), .o1(new_n271));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  tech160nm_fixnrc02aa1n04x5   g177(.a(\b[25] ), .b(\a[26] ), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n272), .c(new_n271), .d(new_n269), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n269), .b(new_n267), .c(new_n191), .d(new_n255), .o1(new_n275));
  nona22aa1n02x5               g180(.a(new_n275), .b(new_n273), .c(new_n272), .out0(new_n276));
  nanp02aa1n03x5               g181(.a(new_n274), .b(new_n276), .o1(\s[26] ));
  inv000aa1n02x5               g182(.a(new_n254), .o1(new_n278));
  nanb02aa1n09x5               g183(.a(new_n273), .b(new_n269), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  nano22aa1n03x7               g185(.a(new_n240), .b(new_n278), .c(new_n280), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n198), .c(new_n150), .d(new_n183), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(\b[25] ), .b(\a[26] ), .o1(new_n283));
  oai022aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n284));
  aoi022aa1n09x5               g189(.a(new_n267), .b(new_n280), .c(new_n283), .d(new_n284), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  xnbna2aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n282), .out0(\s[27] ));
  nand42aa1n03x5               g192(.a(new_n285), .b(new_n282), .o1(new_n288));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  norp02aa1n02x5               g194(.a(\b[27] ), .b(\a[28] ), .o1(new_n290));
  nand42aa1n03x5               g195(.a(\b[27] ), .b(\a[28] ), .o1(new_n291));
  norb02aa1n03x5               g196(.a(new_n291), .b(new_n290), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n289), .c(new_n288), .d(new_n286), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n223), .o1(new_n295));
  nona32aa1n02x4               g200(.a(new_n295), .b(new_n279), .c(new_n254), .d(new_n253), .out0(new_n296));
  aoi012aa1n06x5               g201(.a(new_n296), .b(new_n184), .c(new_n190), .o1(new_n297));
  aoai13aa1n04x5               g202(.a(new_n278), .b(new_n243), .c(new_n228), .d(new_n239), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n284), .b(new_n283), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n279), .c(new_n298), .d(new_n266), .o1(new_n300));
  oaih12aa1n02x5               g205(.a(new_n286), .b(new_n300), .c(new_n297), .o1(new_n301));
  nona22aa1n02x5               g206(.a(new_n301), .b(new_n293), .c(new_n289), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n294), .b(new_n302), .o1(\s[28] ));
  norb02aa1n02x5               g208(.a(new_n286), .b(new_n293), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n300), .c(new_n297), .o1(new_n305));
  oai012aa1n02x5               g210(.a(new_n291), .b(new_n290), .c(new_n289), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[28] ), .b(\a[29] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n02x7               g213(.a(new_n304), .b(new_n285), .c(new_n282), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g217(.a(new_n307), .b(new_n286), .c(new_n292), .out0(new_n313));
  oaih12aa1n02x5               g218(.a(new_n313), .b(new_n300), .c(new_n297), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[29] ), .b(\a[30] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  aobi12aa1n02x7               g222(.a(new_n313), .b(new_n285), .c(new_n282), .out0(new_n318));
  nano22aa1n02x4               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[30] ));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  nano23aa1n02x4               g226(.a(new_n316), .b(new_n307), .c(new_n286), .d(new_n292), .out0(new_n322));
  oaih12aa1n02x5               g227(.a(new_n322), .b(new_n300), .c(new_n297), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n321), .b(new_n323), .c(new_n324), .o1(new_n325));
  aobi12aa1n02x7               g230(.a(new_n322), .b(new_n285), .c(new_n282), .out0(new_n326));
  nano22aa1n02x4               g231(.a(new_n326), .b(new_n321), .c(new_n324), .out0(new_n327));
  norp02aa1n03x5               g232(.a(new_n325), .b(new_n327), .o1(\s[31] ));
  xorb03aa1n02x5               g233(.a(new_n143), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g234(.a(new_n99), .o1(new_n330));
  oai122aa1n02x7               g235(.a(new_n107), .b(new_n143), .c(new_n101), .d(new_n100), .e(new_n330), .o1(new_n331));
  aob012aa1n02x5               g236(.a(new_n331), .b(new_n145), .c(new_n144), .out0(\s[4] ));
  xorb03aa1n02x5               g237(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g238(.a(new_n112), .b(new_n109), .c(new_n113), .o(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi012aa1n02x5               g240(.a(new_n116), .b(new_n112), .c(new_n117), .o1(new_n336));
  oaib12aa1n06x5               g241(.a(new_n336), .b(new_n118), .c(new_n334), .out0(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoai13aa1n02x5               g243(.a(new_n115), .b(new_n110), .c(new_n337), .d(new_n111), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n111), .b(new_n110), .out0(new_n340));
  nanp02aa1n02x5               g245(.a(new_n337), .b(new_n340), .o1(new_n341));
  nona22aa1n02x4               g246(.a(new_n341), .b(new_n115), .c(new_n110), .out0(new_n342));
  nanp02aa1n02x5               g247(.a(new_n342), .b(new_n339), .o1(\s[8] ));
  xorb03aa1n02x5               g248(.a(new_n150), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


