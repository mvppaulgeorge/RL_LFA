// Benchmark "adder" written by ABC on Thu Jul 18 06:29:53 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n141, new_n142, new_n143, new_n144, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n151, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n334, new_n335, new_n336, new_n337,
    new_n339, new_n341, new_n343, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  nor002aa1d32x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n02x4               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n101), .o1(new_n107));
  tech160nm_fioai012aa1n03p5x5 g012(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n108));
  nand42aa1n10x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1d24x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nand42aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nano23aa1n02x4               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  nor002aa1d32x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand02aa1d28x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanb02aa1d24x5               g021(.a(new_n115), .b(new_n116), .out0(new_n117));
  inv000aa1d42x5               g022(.a(new_n117), .o1(new_n118));
  nand03aa1n02x5               g023(.a(new_n113), .b(new_n118), .c(new_n114), .o1(new_n119));
  nano22aa1n02x4               g024(.a(new_n115), .b(new_n109), .c(new_n116), .out0(new_n120));
  nor042aa1n09x5               g025(.a(new_n111), .b(new_n110), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n121), .o1(new_n122));
  inv040aa1n08x5               g027(.a(new_n115), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  aoi013aa1n03x5               g029(.a(new_n124), .b(new_n120), .c(new_n114), .d(new_n122), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n119), .c(new_n107), .d(new_n108), .o1(new_n126));
  nand42aa1n08x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  nor042aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1d28x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oaoi03aa1n09x5               g036(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n132));
  nona23aa1n09x5               g037(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n133));
  oai012aa1n12x5               g038(.a(new_n108), .b(new_n133), .c(new_n132), .o1(new_n134));
  nona23aa1n09x5               g039(.a(new_n109), .b(new_n112), .c(new_n111), .d(new_n110), .out0(new_n135));
  xnrc02aa1n12x5               g040(.a(\b[7] ), .b(\a[8] ), .out0(new_n136));
  nor043aa1n06x5               g041(.a(new_n135), .b(new_n136), .c(new_n117), .o1(new_n137));
  nanb03aa1n06x5               g042(.a(new_n115), .b(new_n116), .c(new_n109), .out0(new_n138));
  inv040aa1n02x5               g043(.a(new_n124), .o1(new_n139));
  oai013aa1n06x5               g044(.a(new_n139), .b(new_n138), .c(new_n136), .d(new_n121), .o1(new_n140));
  norb02aa1n03x4               g045(.a(new_n127), .b(new_n97), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n140), .c(new_n134), .d(new_n137), .o1(new_n142));
  oai022aa1d18x5               g047(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n143));
  nanb03aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n130), .out0(new_n144));
  oai012aa1n02x5               g049(.a(new_n144), .b(new_n128), .c(new_n131), .o1(\s[10] ));
  nano23aa1n06x5               g050(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n127), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n140), .c(new_n134), .d(new_n137), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n148));
  nor002aa1n08x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  nand42aa1d28x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n147), .c(new_n148), .out0(\s[11] ));
  nor042aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  aobi12aa1n06x5               g058(.a(new_n151), .b(new_n147), .c(new_n148), .out0(new_n154));
  nand42aa1d28x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n153), .b(new_n155), .out0(new_n156));
  oai012aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n149), .o1(new_n157));
  nanb02aa1n02x5               g062(.a(new_n154), .b(new_n155), .out0(new_n158));
  oai013aa1n02x4               g063(.a(new_n157), .b(new_n158), .c(new_n149), .d(new_n153), .o1(\s[12] ));
  nano32aa1n03x7               g064(.a(new_n156), .b(new_n151), .c(new_n131), .d(new_n141), .out0(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n140), .c(new_n134), .d(new_n137), .o1(new_n161));
  aoai13aa1n12x5               g066(.a(new_n150), .b(new_n149), .c(new_n143), .d(new_n130), .o1(new_n162));
  oaoi03aa1n12x5               g067(.a(\a[12] ), .b(\b[11] ), .c(new_n162), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nor002aa1d32x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand02aa1d28x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n161), .c(new_n164), .out0(\s[13] ));
  inv000aa1d42x5               g073(.a(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n167), .b(new_n163), .c(new_n126), .d(new_n160), .o1(new_n170));
  norp02aa1n24x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand02aa1d24x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  oai022aa1d18x5               g078(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n174));
  nanb03aa1n02x5               g079(.a(new_n174), .b(new_n170), .c(new_n172), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n173), .c(new_n169), .d(new_n170), .o1(\s[14] ));
  nano23aa1n06x5               g081(.a(new_n165), .b(new_n171), .c(new_n172), .d(new_n166), .out0(new_n177));
  aoai13aa1n03x5               g082(.a(new_n177), .b(new_n163), .c(new_n126), .d(new_n160), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n172), .b(new_n171), .c(new_n165), .o1(new_n179));
  nor042aa1n06x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand42aa1n16x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n178), .c(new_n179), .out0(\s[15] ));
  nona23aa1n09x5               g088(.a(new_n172), .b(new_n166), .c(new_n165), .d(new_n171), .out0(new_n184));
  aoai13aa1n02x7               g089(.a(new_n179), .b(new_n184), .c(new_n161), .d(new_n164), .o1(new_n185));
  nor042aa1n06x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nand02aa1n08x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n180), .c(new_n185), .d(new_n181), .o1(new_n189));
  oai022aa1n02x5               g094(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n182), .o1(new_n191));
  aoai13aa1n02x5               g096(.a(new_n187), .b(new_n191), .c(new_n178), .d(new_n179), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n189), .b(new_n192), .c(new_n190), .o1(\s[16] ));
  nano23aa1n02x5               g098(.a(new_n149), .b(new_n153), .c(new_n155), .d(new_n150), .out0(new_n194));
  nano23aa1n02x5               g099(.a(new_n180), .b(new_n186), .c(new_n187), .d(new_n181), .out0(new_n195));
  nand42aa1n02x5               g100(.a(new_n195), .b(new_n177), .o1(new_n196));
  nano22aa1n03x7               g101(.a(new_n196), .b(new_n146), .c(new_n194), .out0(new_n197));
  aoai13aa1n12x5               g102(.a(new_n197), .b(new_n140), .c(new_n134), .d(new_n137), .o1(new_n198));
  oai012aa1n12x5               g103(.a(new_n162), .b(\b[11] ), .c(\a[12] ), .o1(new_n199));
  nona23aa1n02x5               g104(.a(new_n187), .b(new_n181), .c(new_n180), .d(new_n186), .out0(new_n200));
  nor042aa1n02x5               g105(.a(new_n200), .b(new_n184), .o1(new_n201));
  aoai13aa1n02x7               g106(.a(new_n181), .b(new_n180), .c(new_n174), .d(new_n172), .o1(new_n202));
  oaoi03aa1n06x5               g107(.a(\a[16] ), .b(\b[15] ), .c(new_n202), .o1(new_n203));
  aoi013aa1n09x5               g108(.a(new_n203), .b(new_n199), .c(new_n201), .d(new_n155), .o1(new_n204));
  xorc02aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n198), .c(new_n204), .out0(\s[17] ));
  aobi12aa1n02x5               g111(.a(new_n205), .b(new_n198), .c(new_n204), .out0(new_n207));
  nanp02aa1n12x5               g112(.a(new_n198), .b(new_n204), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n205), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .out0(new_n211));
  oai022aa1n02x5               g116(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n212));
  tech160nm_fiao0012aa1n02p5x5 g117(.a(new_n212), .b(\a[18] ), .c(\b[17] ), .o(new_n213));
  oai022aa1n02x5               g118(.a(new_n210), .b(new_n211), .c(new_n213), .d(new_n207), .o1(\s[18] ));
  inv040aa1d30x5               g119(.a(\a[17] ), .o1(new_n215));
  inv040aa1d32x5               g120(.a(\a[18] ), .o1(new_n216));
  xroi22aa1d06x4               g121(.a(new_n215), .b(\b[16] ), .c(new_n216), .d(\b[17] ), .out0(new_n217));
  inv000aa1n02x5               g122(.a(new_n217), .o1(new_n218));
  inv000aa1n03x5               g123(.a(new_n209), .o1(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n219), .o1(new_n220));
  inv000aa1n02x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n218), .c(new_n198), .d(new_n204), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[19] ), .b(\b[18] ), .out0(new_n223));
  aoi112aa1n03x4               g128(.a(new_n223), .b(new_n220), .c(new_n208), .d(new_n217), .o1(new_n224));
  tech160nm_fiaoi012aa1n02p5x5 g129(.a(new_n224), .b(new_n222), .c(new_n223), .o1(\s[19] ));
  xnrc02aa1n02x5               g130(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g131(.a(\a[19] ), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\b[18] ), .o1(new_n228));
  oaoi03aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n222), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[20] ), .b(\b[19] ), .out0(new_n230));
  nanp02aa1n02x5               g135(.a(new_n222), .b(new_n223), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  inv040aa1d32x5               g137(.a(\a[20] ), .o1(new_n233));
  aboi22aa1n12x5               g138(.a(\b[19] ), .b(new_n233), .c(new_n227), .d(new_n228), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n231), .b(new_n232), .c(new_n234), .o1(new_n235));
  oaih12aa1n02x5               g140(.a(new_n235), .b(new_n229), .c(new_n230), .o1(\s[20] ));
  xroi22aa1d04x5               g141(.a(new_n227), .b(\b[18] ), .c(new_n233), .d(\b[19] ), .out0(new_n237));
  and002aa1n12x5               g142(.a(new_n237), .b(new_n217), .o(new_n238));
  nor002aa1n04x5               g143(.a(\b[17] ), .b(\a[18] ), .o1(new_n239));
  aoi022aa1d24x5               g144(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n240));
  oai012aa1n06x5               g145(.a(new_n240), .b(new_n239), .c(new_n209), .o1(new_n241));
  aoi022aa1n09x5               g146(.a(new_n241), .b(new_n234), .c(\a[20] ), .d(\b[19] ), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[21] ), .b(\b[20] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n242), .c(new_n208), .d(new_n238), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n242), .c(new_n208), .d(new_n238), .o1(new_n245));
  norb02aa1n03x4               g150(.a(new_n244), .b(new_n245), .out0(\s[21] ));
  orn002aa1n02x5               g151(.a(\a[21] ), .b(\b[20] ), .o(new_n247));
  xnrc02aa1n12x5               g152(.a(\b[21] ), .b(\a[22] ), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  and002aa1n02x5               g154(.a(\b[21] ), .b(\a[22] ), .o(new_n250));
  oai022aa1n02x5               g155(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n251));
  nona22aa1n02x5               g156(.a(new_n244), .b(new_n250), .c(new_n251), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n249), .c(new_n247), .d(new_n244), .o1(\s[22] ));
  nanp02aa1n02x5               g158(.a(\b[20] ), .b(\a[21] ), .o1(new_n254));
  nano22aa1n12x5               g159(.a(new_n248), .b(new_n247), .c(new_n254), .out0(new_n255));
  nano22aa1n02x4               g160(.a(new_n218), .b(new_n237), .c(new_n255), .out0(new_n256));
  oaoi03aa1n02x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n257));
  tech160nm_fiao0012aa1n02p5x5 g162(.a(new_n257), .b(new_n242), .c(new_n255), .o(new_n258));
  xorc02aa1n12x5               g163(.a(\a[23] ), .b(\b[22] ), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n208), .d(new_n256), .o1(new_n260));
  aoi112aa1n02x5               g165(.a(new_n259), .b(new_n258), .c(new_n208), .d(new_n256), .o1(new_n261));
  norb02aa1n03x4               g166(.a(new_n260), .b(new_n261), .out0(\s[23] ));
  nor042aa1d18x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  inv040aa1n06x5               g168(.a(new_n263), .o1(new_n264));
  xorc02aa1n12x5               g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  and002aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o(new_n266));
  oai022aa1n02x5               g171(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n267));
  nona22aa1n02x5               g172(.a(new_n260), .b(new_n266), .c(new_n267), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n265), .c(new_n264), .d(new_n260), .o1(\s[24] ));
  nand02aa1n04x5               g174(.a(new_n265), .b(new_n259), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  nano32aa1n02x4               g176(.a(new_n218), .b(new_n271), .c(new_n237), .d(new_n255), .out0(new_n272));
  tech160nm_fioaoi03aa1n03p5x5 g177(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .o1(new_n273));
  inv030aa1d32x5               g178(.a(new_n273), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n271), .b(new_n257), .c(new_n242), .d(new_n255), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(new_n275), .b(new_n274), .o1(new_n276));
  tech160nm_fixorc02aa1n04x5   g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n208), .d(new_n272), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n277), .b(new_n276), .c(new_n208), .d(new_n272), .o1(new_n279));
  norb02aa1n03x4               g184(.a(new_n278), .b(new_n279), .out0(\s[25] ));
  nor042aa1d18x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  inv020aa1n04x5               g186(.a(new_n281), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  oai022aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(\a[26] ), .c(\b[25] ), .o1(new_n285));
  nand42aa1n03x5               g190(.a(new_n278), .b(new_n285), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n283), .c(new_n282), .d(new_n278), .o1(\s[26] ));
  nanp03aa1n03x5               g192(.a(new_n255), .b(new_n259), .c(new_n265), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n283), .b(new_n277), .o1(new_n289));
  nona22aa1n02x4               g194(.a(new_n238), .b(new_n288), .c(new_n289), .out0(new_n290));
  inv040aa1n02x5               g195(.a(new_n290), .o1(new_n291));
  tech160nm_fioaoi03aa1n02p5x5 g196(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .o1(new_n292));
  inv030aa1n02x5               g197(.a(new_n292), .o1(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n289), .c(new_n275), .d(new_n274), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[27] ), .b(\b[26] ), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n294), .c(new_n208), .d(new_n291), .o1(new_n296));
  tech160nm_fiaoi012aa1n03p5x5 g201(.a(new_n290), .b(new_n198), .c(new_n204), .o1(new_n297));
  norp03aa1n02x5               g202(.a(new_n297), .b(new_n294), .c(new_n295), .o1(new_n298));
  norb02aa1n03x4               g203(.a(new_n296), .b(new_n298), .out0(\s[27] ));
  norp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  oai022aa1d24x5               g207(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(\a[28] ), .c(\b[27] ), .o1(new_n304));
  tech160nm_finand02aa1n03p5x5 g209(.a(new_n296), .b(new_n304), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n302), .c(new_n301), .d(new_n296), .o1(\s[28] ));
  xorc02aa1n12x5               g211(.a(\a[29] ), .b(\b[28] ), .out0(new_n307));
  and002aa1n02x5               g212(.a(new_n302), .b(new_n295), .o(new_n308));
  inv000aa1d42x5               g213(.a(\b[27] ), .o1(new_n309));
  oaib12aa1n09x5               g214(.a(new_n303), .b(new_n309), .c(\a[28] ), .out0(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  oaoi13aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n297), .d(new_n294), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n307), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n308), .b(new_n294), .c(new_n208), .d(new_n291), .o1(new_n314));
  nona22aa1n03x5               g219(.a(new_n314), .b(new_n311), .c(new_n313), .out0(new_n315));
  oai012aa1n03x5               g220(.a(new_n315), .b(new_n312), .c(new_n307), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g222(.a(new_n313), .b(new_n295), .c(new_n302), .out0(new_n318));
  tech160nm_fioaoi03aa1n03p5x5 g223(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .o1(new_n319));
  oaoi13aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n297), .d(new_n294), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n318), .b(new_n294), .c(new_n208), .d(new_n291), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n321), .b(new_n319), .out0(new_n323));
  nand02aa1n02x5               g228(.a(new_n322), .b(new_n323), .o1(new_n324));
  tech160nm_fioai012aa1n02p5x5 g229(.a(new_n324), .b(new_n320), .c(new_n321), .o1(\s[30] ));
  nano32aa1n02x4               g230(.a(new_n313), .b(new_n321), .c(new_n295), .d(new_n302), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n294), .c(new_n208), .d(new_n291), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n328));
  xorc02aa1n02x5               g233(.a(\a[31] ), .b(\b[30] ), .out0(new_n329));
  oai012aa1n02x5               g234(.a(new_n329), .b(\b[29] ), .c(\a[30] ), .o1(new_n330));
  aoi012aa1n02x5               g235(.a(new_n330), .b(new_n319), .c(new_n321), .o1(new_n331));
  nand42aa1n02x5               g236(.a(new_n327), .b(new_n331), .o1(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n329), .c(new_n327), .d(new_n328), .o1(\s[31] ));
  aoi022aa1n02x5               g238(.a(new_n99), .b(new_n98), .c(\a[1] ), .d(\b[0] ), .o1(new_n334));
  oaib12aa1n02x5               g239(.a(new_n334), .b(new_n99), .c(\a[2] ), .out0(new_n335));
  norb02aa1n02x5               g240(.a(new_n105), .b(new_n104), .out0(new_n336));
  aboi22aa1n03x5               g241(.a(new_n104), .b(new_n105), .c(new_n98), .d(new_n99), .out0(new_n337));
  aoi022aa1n02x5               g242(.a(new_n101), .b(new_n336), .c(new_n335), .d(new_n337), .o1(\s[3] ));
  oaoi03aa1n02x5               g243(.a(\a[3] ), .b(\b[2] ), .c(new_n132), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanb02aa1n02x5               g245(.a(new_n111), .b(new_n112), .out0(new_n341));
  xobna2aa1n03x5               g246(.a(new_n341), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  aoi012aa1n02x5               g247(.a(new_n111), .b(new_n134), .c(new_n112), .o1(new_n343));
  xnrb03aa1n02x5               g248(.a(new_n343), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  ao0022aa1n03x5               g249(.a(new_n134), .b(new_n113), .c(new_n122), .d(new_n109), .o(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp02aa1n02x5               g251(.a(new_n345), .b(new_n118), .o1(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n114), .b(new_n347), .c(new_n123), .out0(\s[8] ));
  aoi112aa1n02x5               g253(.a(new_n141), .b(new_n140), .c(new_n134), .d(new_n137), .o1(new_n349));
  aoi012aa1n02x5               g254(.a(new_n349), .b(new_n126), .c(new_n141), .o1(\s[9] ));
endmodule


