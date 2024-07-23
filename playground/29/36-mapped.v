// Benchmark "adder" written by ABC on Thu Jul 18 03:08:54 2024

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
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n338,
    new_n339, new_n342, new_n344, new_n346, new_n347;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fioaoi03aa1n03p5x5 g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor002aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1n10x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand22aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n103), .b(new_n106), .c(new_n105), .d(new_n104), .out0(new_n107));
  tech160nm_fioai012aa1n02p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  xorc02aa1n02x5               g014(.a(\a[6] ), .b(\b[5] ), .out0(new_n110));
  xorc02aa1n02x5               g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  tech160nm_fixorc02aa1n05x5   g017(.a(\a[7] ), .b(\b[6] ), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(new_n113), .b(new_n112), .o1(new_n114));
  nano22aa1n03x7               g019(.a(new_n114), .b(new_n110), .c(new_n111), .out0(new_n115));
  nanp02aa1n03x5               g020(.a(new_n115), .b(new_n109), .o1(new_n116));
  norp02aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n117), .o1(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  inv000aa1n02x5               g024(.a(new_n119), .o1(new_n120));
  nand02aa1n04x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  xnrc02aa1n03x5               g026(.a(\b[7] ), .b(\a[8] ), .out0(new_n122));
  xnrc02aa1n12x5               g027(.a(\b[6] ), .b(\a[7] ), .out0(new_n123));
  oai022aa1n02x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  nano23aa1n02x4               g029(.a(new_n123), .b(new_n122), .c(new_n124), .d(new_n121), .out0(new_n125));
  nano22aa1n03x7               g030(.a(new_n125), .b(new_n118), .c(new_n120), .out0(new_n126));
  nand42aa1n08x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(new_n128));
  inv000aa1n02x5               g033(.a(new_n128), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n98), .b(new_n129), .c(new_n116), .d(new_n126), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1d28x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  oao003aa1n02x5               g040(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n136));
  nano23aa1n02x4               g041(.a(new_n105), .b(new_n104), .c(new_n106), .d(new_n103), .out0(new_n137));
  aobi12aa1n03x5               g042(.a(new_n108), .b(new_n137), .c(new_n136), .out0(new_n138));
  xnrc02aa1n02x5               g043(.a(\b[5] ), .b(\a[6] ), .out0(new_n139));
  nona23aa1n02x4               g044(.a(new_n113), .b(new_n111), .c(new_n139), .d(new_n122), .out0(new_n140));
  oaoi13aa1n06x5               g045(.a(new_n129), .b(new_n126), .c(new_n140), .d(new_n138), .o1(new_n141));
  oai012aa1d24x5               g046(.a(new_n133), .b(new_n97), .c(new_n132), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  nor002aa1n16x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nand42aa1n04x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  norb02aa1n06x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  aoai13aa1n03x5               g051(.a(new_n146), .b(new_n143), .c(new_n141), .d(new_n135), .o1(new_n147));
  oai012aa1n04x7               g052(.a(new_n126), .b(new_n140), .c(new_n138), .o1(new_n148));
  aoi113aa1n02x5               g053(.a(new_n146), .b(new_n143), .c(new_n148), .d(new_n128), .e(new_n135), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n147), .b(new_n149), .out0(\s[11] ));
  inv000aa1d42x5               g055(.a(new_n144), .o1(new_n151));
  tech160nm_fixnrc02aa1n05x5   g056(.a(\b[11] ), .b(\a[12] ), .out0(new_n152));
  xobna2aa1n03x5               g057(.a(new_n152), .b(new_n147), .c(new_n151), .out0(\s[12] ));
  nano23aa1d15x5               g058(.a(new_n132), .b(new_n97), .c(new_n127), .d(new_n133), .out0(new_n154));
  nanb03aa1d24x5               g059(.a(new_n152), .b(new_n154), .c(new_n146), .out0(new_n155));
  oai022aa1d18x5               g060(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n145), .b(new_n144), .c(new_n156), .d(new_n133), .o1(new_n157));
  oaoi03aa1n12x5               g062(.a(\a[12] ), .b(\b[11] ), .c(new_n157), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n155), .c(new_n116), .d(new_n126), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d28x5               g066(.a(\a[13] ), .o1(new_n162));
  inv040aa1d32x5               g067(.a(\b[12] ), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(new_n162), .b(new_n163), .c(new_n160), .o1(new_n164));
  xnrb03aa1n03x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g070(.a(new_n155), .o1(new_n166));
  nor042aa1n02x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  tech160nm_finand02aa1n03p5x5 g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1n20x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nano23aa1n06x5               g075(.a(new_n167), .b(new_n169), .c(new_n170), .d(new_n168), .out0(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n158), .c(new_n148), .d(new_n166), .o1(new_n172));
  aoai13aa1n12x5               g077(.a(new_n170), .b(new_n169), .c(new_n162), .d(new_n163), .o1(new_n173));
  xorc02aa1n12x5               g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  orn002aa1n02x5               g080(.a(\a[15] ), .b(\b[14] ), .o(new_n176));
  aobi12aa1n02x5               g081(.a(new_n174), .b(new_n172), .c(new_n173), .out0(new_n177));
  xorc02aa1n12x5               g082(.a(\a[16] ), .b(\b[15] ), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nano22aa1n03x5               g084(.a(new_n177), .b(new_n176), .c(new_n179), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n173), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n174), .b(new_n181), .c(new_n160), .d(new_n171), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n179), .b(new_n182), .c(new_n176), .o1(new_n183));
  norp02aa1n02x5               g088(.a(new_n183), .b(new_n180), .o1(\s[16] ));
  inv000aa1n02x5               g089(.a(new_n121), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[5] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\a[6] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[4] ), .o1(new_n188));
  aboi22aa1n03x5               g093(.a(\b[5] ), .b(new_n187), .c(new_n186), .d(new_n188), .out0(new_n189));
  nona23aa1n02x4               g094(.a(new_n113), .b(new_n112), .c(new_n189), .d(new_n185), .out0(new_n190));
  nona22aa1n02x4               g095(.a(new_n190), .b(new_n119), .c(new_n117), .out0(new_n191));
  nand03aa1n04x5               g096(.a(new_n171), .b(new_n174), .c(new_n178), .o1(new_n192));
  nor042aa1n09x5               g097(.a(new_n155), .b(new_n192), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n191), .c(new_n109), .d(new_n115), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[15] ), .b(\a[16] ), .o1(new_n195));
  and002aa1n02x5               g100(.a(\b[14] ), .b(\a[15] ), .o(new_n196));
  orn002aa1n02x5               g101(.a(\a[16] ), .b(\b[15] ), .o(new_n197));
  aoai13aa1n09x5               g102(.a(new_n197), .b(new_n196), .c(new_n173), .d(new_n176), .o1(new_n198));
  aboi22aa1n12x5               g103(.a(new_n192), .b(new_n158), .c(new_n198), .d(new_n195), .out0(new_n199));
  tech160nm_fixnrc02aa1n05x5   g104(.a(\b[16] ), .b(\a[17] ), .out0(new_n200));
  xobna2aa1n03x5               g105(.a(new_n200), .b(new_n194), .c(new_n199), .out0(\s[17] ));
  inv000aa1d42x5               g106(.a(\a[18] ), .o1(new_n202));
  nanp02aa1n12x5               g107(.a(new_n194), .b(new_n199), .o1(new_n203));
  norp02aa1n02x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  aoib12aa1n06x5               g109(.a(new_n204), .b(new_n203), .c(new_n200), .out0(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[17] ), .c(new_n202), .out0(\s[18] ));
  xorc02aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .out0(new_n207));
  norb02aa1n03x5               g112(.a(new_n207), .b(new_n200), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  oai022aa1n04x7               g115(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n211));
  and002aa1n02x5               g116(.a(new_n211), .b(new_n210), .o(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n209), .c(new_n194), .d(new_n199), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nanp02aa1n04x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nor042aa1n06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand02aa1n08x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoi112aa1n03x4               g126(.a(new_n217), .b(new_n221), .c(new_n214), .d(new_n218), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  nanb02aa1n02x5               g128(.a(\a[19] ), .b(new_n223), .out0(new_n224));
  norb02aa1n02x5               g129(.a(new_n218), .b(new_n217), .out0(new_n225));
  nand02aa1n02x5               g130(.a(new_n214), .b(new_n225), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n221), .o1(new_n227));
  tech160nm_fiaoi012aa1n03p5x5 g132(.a(new_n227), .b(new_n226), .c(new_n224), .o1(new_n228));
  nor042aa1n03x5               g133(.a(new_n228), .b(new_n222), .o1(\s[20] ));
  nano22aa1n03x7               g134(.a(new_n219), .b(new_n218), .c(new_n220), .out0(new_n230));
  nona23aa1n06x5               g135(.a(new_n230), .b(new_n207), .c(new_n200), .d(new_n217), .out0(new_n231));
  nanb03aa1n09x5               g136(.a(new_n219), .b(new_n220), .c(new_n218), .out0(new_n232));
  nand23aa1n03x5               g137(.a(new_n211), .b(new_n224), .c(new_n210), .o1(new_n233));
  aoi012aa1n12x5               g138(.a(new_n219), .b(new_n217), .c(new_n220), .o1(new_n234));
  oai012aa1n18x5               g139(.a(new_n234), .b(new_n233), .c(new_n232), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n231), .c(new_n194), .d(new_n199), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[21] ), .b(\a[22] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoi112aa1n03x4               g148(.a(new_n239), .b(new_n243), .c(new_n237), .d(new_n241), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n239), .o1(new_n245));
  nand02aa1n02x5               g150(.a(new_n237), .b(new_n241), .o1(new_n246));
  aoi012aa1n03x5               g151(.a(new_n242), .b(new_n246), .c(new_n245), .o1(new_n247));
  norp02aa1n03x5               g152(.a(new_n247), .b(new_n244), .o1(\s[22] ));
  nor042aa1n06x5               g153(.a(new_n242), .b(new_n240), .o1(new_n249));
  nona23aa1n06x5               g154(.a(new_n208), .b(new_n249), .c(new_n232), .d(new_n217), .out0(new_n250));
  oao003aa1n12x5               g155(.a(\a[22] ), .b(\b[21] ), .c(new_n245), .carry(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n235), .c(new_n249), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n250), .c(new_n194), .d(new_n199), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  tech160nm_fixorc02aa1n05x5   g161(.a(\a[23] ), .b(\b[22] ), .out0(new_n257));
  xorc02aa1n12x5               g162(.a(\a[24] ), .b(\b[23] ), .out0(new_n258));
  aoi112aa1n03x4               g163(.a(new_n256), .b(new_n258), .c(new_n254), .d(new_n257), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n256), .o1(new_n260));
  nand02aa1n02x5               g165(.a(new_n254), .b(new_n257), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n258), .o1(new_n262));
  aoi012aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .o1(new_n263));
  nor002aa1n02x5               g168(.a(new_n263), .b(new_n259), .o1(\s[24] ));
  nano32aa1n03x7               g169(.a(new_n231), .b(new_n258), .c(new_n249), .d(new_n257), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  oai012aa1n02x5               g171(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n211), .b(new_n267), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n234), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n249), .b(new_n269), .c(new_n268), .d(new_n230), .o1(new_n270));
  and002aa1n12x5               g175(.a(new_n258), .b(new_n257), .o(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n260), .carry(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n270), .d(new_n251), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n266), .c(new_n194), .d(new_n199), .o1(new_n276));
  xorb03aa1n02x5               g181(.a(new_n276), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  tech160nm_fixorc02aa1n03p5x5 g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  xorc02aa1n12x5               g184(.a(\a[26] ), .b(\b[25] ), .out0(new_n280));
  aoi112aa1n03x4               g185(.a(new_n278), .b(new_n280), .c(new_n276), .d(new_n279), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n278), .o1(new_n282));
  nand02aa1n02x5               g187(.a(new_n276), .b(new_n279), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n280), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n284), .b(new_n283), .c(new_n282), .o1(new_n285));
  nor002aa1n02x5               g190(.a(new_n285), .b(new_n281), .o1(\s[26] ));
  inv000aa1d42x5               g191(.a(\a[12] ), .o1(new_n287));
  inv000aa1d42x5               g192(.a(\b[11] ), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(new_n142), .b(new_n151), .o1(new_n290));
  oai112aa1n02x5               g195(.a(new_n290), .b(new_n145), .c(new_n288), .d(new_n287), .o1(new_n291));
  nand42aa1n02x5               g196(.a(new_n198), .b(new_n195), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n192), .c(new_n291), .d(new_n289), .o1(new_n293));
  and002aa1n02x5               g198(.a(new_n280), .b(new_n279), .o(new_n294));
  inv000aa1n02x5               g199(.a(new_n294), .o1(new_n295));
  nano23aa1n06x5               g200(.a(new_n295), .b(new_n231), .c(new_n271), .d(new_n249), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n293), .c(new_n148), .d(new_n193), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .carry(new_n298));
  aobi12aa1n06x5               g203(.a(new_n298), .b(new_n274), .c(new_n294), .out0(new_n299));
  xorc02aa1n12x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  xnbna2aa1n03x5               g205(.a(new_n300), .b(new_n299), .c(new_n297), .out0(\s[27] ));
  nor042aa1n03x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n302), .o1(new_n303));
  aobi12aa1n02x7               g208(.a(new_n300), .b(new_n299), .c(new_n297), .out0(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n304), .b(new_n303), .c(new_n305), .out0(new_n306));
  aoai13aa1n04x5               g211(.a(new_n271), .b(new_n252), .c(new_n235), .d(new_n249), .o1(new_n307));
  aoai13aa1n06x5               g212(.a(new_n298), .b(new_n295), .c(new_n307), .d(new_n273), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n300), .b(new_n308), .c(new_n203), .d(new_n296), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n305), .b(new_n309), .c(new_n303), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n306), .o1(\s[28] ));
  xnrc02aa1n02x5               g216(.a(\b[28] ), .b(\a[29] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n300), .b(new_n305), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n308), .c(new_n203), .d(new_n296), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n303), .carry(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n312), .b(new_n314), .c(new_n315), .o1(new_n316));
  aobi12aa1n02x7               g221(.a(new_n313), .b(new_n299), .c(new_n297), .out0(new_n317));
  nano22aa1n02x4               g222(.a(new_n317), .b(new_n312), .c(new_n315), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g225(.a(new_n300), .b(new_n312), .c(new_n305), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n308), .c(new_n203), .d(new_n296), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[29] ), .b(\a[30] ), .out0(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n324), .b(new_n322), .c(new_n323), .o1(new_n325));
  aobi12aa1n02x7               g230(.a(new_n321), .b(new_n299), .c(new_n297), .out0(new_n326));
  nano22aa1n02x4               g231(.a(new_n326), .b(new_n323), .c(new_n324), .out0(new_n327));
  nor002aa1n02x5               g232(.a(new_n325), .b(new_n327), .o1(\s[30] ));
  norb02aa1n02x5               g233(.a(new_n321), .b(new_n324), .out0(new_n329));
  aobi12aa1n02x7               g234(.a(new_n329), .b(new_n299), .c(new_n297), .out0(new_n330));
  oao003aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[30] ), .b(\a[31] ), .out0(new_n332));
  nano22aa1n02x4               g237(.a(new_n330), .b(new_n331), .c(new_n332), .out0(new_n333));
  aoai13aa1n03x5               g238(.a(new_n329), .b(new_n308), .c(new_n203), .d(new_n296), .o1(new_n334));
  tech160nm_fiaoi012aa1n02p5x5 g239(.a(new_n332), .b(new_n334), .c(new_n331), .o1(new_n335));
  norp02aa1n03x5               g240(.a(new_n335), .b(new_n333), .o1(\s[31] ));
  xnrb03aa1n02x5               g241(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb03aa1n02x5               g242(.a(new_n105), .b(new_n136), .c(new_n106), .out0(new_n338));
  aoib12aa1n02x5               g243(.a(new_n105), .b(new_n103), .c(new_n104), .out0(new_n339));
  aboi22aa1n03x5               g244(.a(new_n104), .b(new_n109), .c(new_n338), .d(new_n339), .out0(\s[4] ));
  xorb03aa1n02x5               g245(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g246(.a(new_n186), .b(new_n188), .c(new_n109), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[5] ), .c(new_n187), .out0(\s[6] ));
  nanp02aa1n02x5               g248(.a(new_n342), .b(new_n110), .o1(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n123), .b(new_n344), .c(new_n121), .out0(\s[7] ));
  orn002aa1n02x5               g250(.a(\a[7] ), .b(\b[6] ), .o(new_n346));
  nona22aa1n02x4               g251(.a(new_n344), .b(new_n123), .c(new_n185), .out0(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n112), .b(new_n347), .c(new_n346), .out0(\s[8] ));
  xnbna2aa1n03x5               g253(.a(new_n128), .b(new_n116), .c(new_n126), .out0(\s[9] ));
endmodule


