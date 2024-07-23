// Benchmark "adder" written by ABC on Thu Jul 18 00:59:17 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n06x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  nand02aa1d08x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n03x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  tech160nm_fixorc02aa1n02p5x5 g008(.a(\a[3] ), .b(\b[2] ), .out0(new_n104));
  nand03aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  inv040aa1d32x5               g010(.a(\a[3] ), .o1(new_n106));
  inv040aa1d28x5               g011(.a(\b[2] ), .o1(new_n107));
  nand02aa1d16x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  oaoi03aa1n09x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .o1(new_n109));
  inv040aa1n02x5               g014(.a(new_n109), .o1(new_n110));
  nor042aa1d18x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d28x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor042aa1d18x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n09x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nor042aa1n12x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand02aa1d28x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nor042aa1d18x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nand02aa1n12x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nano23aa1n02x5               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nand02aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  aoi012aa1d18x5               g026(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n122));
  inv030aa1n04x5               g027(.a(new_n122), .o1(new_n123));
  tech160nm_fioai012aa1n05x5   g028(.a(new_n112), .b(new_n113), .c(new_n111), .o1(new_n124));
  aobi12aa1n12x5               g029(.a(new_n124), .b(new_n115), .c(new_n123), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n121), .c(new_n105), .d(new_n110), .o1(new_n126));
  nand02aa1d24x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand02aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nano23aa1d15x5               g036(.a(new_n97), .b(new_n130), .c(new_n131), .d(new_n127), .out0(new_n132));
  aoi012aa1n09x5               g037(.a(new_n130), .b(new_n97), .c(new_n131), .o1(new_n133));
  inv000aa1n03x5               g038(.a(new_n133), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n16x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoai13aa1n06x5               g042(.a(new_n137), .b(new_n134), .c(new_n126), .d(new_n132), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n137), .b(new_n134), .c(new_n126), .d(new_n132), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  tech160nm_fioai012aa1n03p5x5 g045(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  aoi012aa1d18x5               g047(.a(new_n98), .b(new_n100), .c(new_n101), .o1(new_n143));
  inv000aa1d42x5               g048(.a(\a[4] ), .o1(new_n144));
  nanb02aa1n12x5               g049(.a(\b[3] ), .b(new_n144), .out0(new_n145));
  nanp02aa1n04x5               g050(.a(\b[3] ), .b(\a[4] ), .o1(new_n146));
  nanp02aa1n06x5               g051(.a(new_n145), .b(new_n146), .o1(new_n147));
  nanp02aa1n06x5               g052(.a(\b[2] ), .b(\a[3] ), .o1(new_n148));
  nand02aa1n04x5               g053(.a(new_n108), .b(new_n148), .o1(new_n149));
  norp03aa1n04x5               g054(.a(new_n143), .b(new_n147), .c(new_n149), .o1(new_n150));
  nanb02aa1n12x5               g055(.a(new_n111), .b(new_n112), .out0(new_n151));
  nanb02aa1n12x5               g056(.a(new_n113), .b(new_n114), .out0(new_n152));
  nona23aa1d16x5               g057(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n153));
  nor043aa1d12x5               g058(.a(new_n153), .b(new_n152), .c(new_n151), .o1(new_n154));
  oai012aa1n06x5               g059(.a(new_n154), .b(new_n150), .c(new_n109), .o1(new_n155));
  nor002aa1d32x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  nand42aa1n16x5               g061(.a(\b[11] ), .b(\a[12] ), .o1(new_n157));
  nano23aa1n09x5               g062(.a(new_n135), .b(new_n156), .c(new_n157), .d(new_n136), .out0(new_n158));
  nand22aa1n12x5               g063(.a(new_n158), .b(new_n132), .o1(new_n159));
  tech160nm_fioai012aa1n03p5x5 g064(.a(new_n157), .b(new_n156), .c(new_n135), .o1(new_n160));
  aobi12aa1n03x5               g065(.a(new_n160), .b(new_n158), .c(new_n134), .out0(new_n161));
  aoai13aa1n03x5               g066(.a(new_n161), .b(new_n159), .c(new_n155), .d(new_n125), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand02aa1n12x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n164), .b(new_n162), .c(new_n165), .o1(new_n166));
  xnrb03aa1n03x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n09x5               g072(.a(new_n110), .b(new_n143), .c(new_n147), .d(new_n149), .o1(new_n168));
  oai013aa1n06x5               g073(.a(new_n124), .b(new_n122), .c(new_n151), .d(new_n152), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n159), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n168), .d(new_n154), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand02aa1d28x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nona23aa1n09x5               g078(.a(new_n173), .b(new_n165), .c(new_n164), .d(new_n172), .out0(new_n174));
  aoi012aa1d24x5               g079(.a(new_n172), .b(new_n164), .c(new_n173), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n175), .b(new_n174), .c(new_n171), .d(new_n161), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nano23aa1n06x5               g084(.a(new_n164), .b(new_n172), .c(new_n173), .d(new_n165), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n175), .o1(new_n181));
  nand02aa1n10x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nanb02aa1n18x5               g087(.a(new_n178), .b(new_n182), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n184), .b(new_n181), .c(new_n162), .d(new_n180), .o1(new_n185));
  nor002aa1d24x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nand42aa1n16x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nanb02aa1n18x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  tech160nm_fiaoi012aa1n02p5x5 g093(.a(new_n188), .b(new_n185), .c(new_n179), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n188), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n178), .b(new_n190), .c(new_n176), .d(new_n184), .o1(new_n191));
  norp02aa1n03x5               g096(.a(new_n189), .b(new_n191), .o1(\s[16] ));
  nona23aa1n09x5               g097(.a(new_n157), .b(new_n136), .c(new_n135), .d(new_n156), .out0(new_n193));
  nona23aa1d18x5               g098(.a(new_n187), .b(new_n182), .c(new_n178), .d(new_n186), .out0(new_n194));
  nona23aa1n09x5               g099(.a(new_n180), .b(new_n132), .c(new_n194), .d(new_n193), .out0(new_n195));
  oai012aa1n09x5               g100(.a(new_n160), .b(new_n193), .c(new_n133), .o1(new_n196));
  nor042aa1n04x5               g101(.a(new_n194), .b(new_n174), .o1(new_n197));
  tech160nm_fioai012aa1n03p5x5 g102(.a(new_n187), .b(new_n186), .c(new_n178), .o1(new_n198));
  oai012aa1n18x5               g103(.a(new_n198), .b(new_n194), .c(new_n175), .o1(new_n199));
  aoi012aa1d24x5               g104(.a(new_n199), .b(new_n196), .c(new_n197), .o1(new_n200));
  aoai13aa1n12x5               g105(.a(new_n200), .b(new_n195), .c(new_n155), .d(new_n125), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g107(.a(\a[18] ), .o1(new_n203));
  inv040aa1d30x5               g108(.a(\a[17] ), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\b[16] ), .o1(new_n205));
  oaoi03aa1n03x5               g110(.a(new_n204), .b(new_n205), .c(new_n201), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[17] ), .c(new_n203), .out0(\s[18] ));
  nona22aa1n09x5               g112(.a(new_n180), .b(new_n183), .c(new_n188), .out0(new_n208));
  nor042aa1n06x5               g113(.a(new_n208), .b(new_n159), .o1(new_n209));
  aoai13aa1n09x5               g114(.a(new_n209), .b(new_n169), .c(new_n168), .d(new_n154), .o1(new_n210));
  xroi22aa1d06x4               g115(.a(new_n204), .b(\b[16] ), .c(new_n203), .d(\b[17] ), .out0(new_n211));
  inv000aa1n06x5               g116(.a(new_n211), .o1(new_n212));
  nor042aa1d18x5               g117(.a(\b[16] ), .b(\a[17] ), .o1(new_n213));
  nor042aa1n04x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand02aa1d24x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  aoi012aa1n12x5               g120(.a(new_n214), .b(new_n213), .c(new_n215), .o1(new_n216));
  aoai13aa1n04x5               g121(.a(new_n216), .b(new_n212), .c(new_n210), .d(new_n200), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1d18x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  inv030aa1n03x5               g125(.a(new_n220), .o1(new_n221));
  inv040aa1n08x5               g126(.a(new_n216), .o1(new_n222));
  xorc02aa1n12x5               g127(.a(\a[19] ), .b(\b[18] ), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n222), .c(new_n201), .d(new_n211), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[19] ), .b(\a[20] ), .out0(new_n225));
  aoi012aa1n02x7               g130(.a(new_n225), .b(new_n224), .c(new_n221), .o1(new_n226));
  xorc02aa1n12x5               g131(.a(\a[20] ), .b(\b[19] ), .out0(new_n227));
  aoi112aa1n03x4               g132(.a(new_n220), .b(new_n227), .c(new_n217), .d(new_n223), .o1(new_n228));
  nor002aa1n02x5               g133(.a(new_n226), .b(new_n228), .o1(\s[20] ));
  xnrc02aa1n06x5               g134(.a(\b[18] ), .b(\a[19] ), .out0(new_n230));
  nor042aa1n06x5               g135(.a(new_n225), .b(new_n230), .o1(new_n231));
  nand22aa1n09x5               g136(.a(new_n211), .b(new_n231), .o1(new_n232));
  oaoi03aa1n12x5               g137(.a(\a[20] ), .b(\b[19] ), .c(new_n221), .o1(new_n233));
  aoi013aa1n09x5               g138(.a(new_n233), .b(new_n222), .c(new_n223), .d(new_n227), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n232), .c(new_n210), .d(new_n200), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n232), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n234), .o1(new_n240));
  nand42aa1d28x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n237), .b(new_n241), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n240), .c(new_n201), .d(new_n239), .o1(new_n244));
  nor002aa1n16x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nand42aa1n20x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  aoi012aa1n03x5               g152(.a(new_n247), .b(new_n244), .c(new_n238), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n247), .o1(new_n249));
  aoi112aa1n02x7               g154(.a(new_n237), .b(new_n249), .c(new_n235), .d(new_n243), .o1(new_n250));
  norp02aa1n03x5               g155(.a(new_n248), .b(new_n250), .o1(\s[22] ));
  nano23aa1n02x5               g156(.a(new_n237), .b(new_n245), .c(new_n246), .d(new_n241), .out0(new_n252));
  nano22aa1n02x4               g157(.a(new_n212), .b(new_n231), .c(new_n252), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  nona23aa1n09x5               g159(.a(new_n246), .b(new_n241), .c(new_n237), .d(new_n245), .out0(new_n255));
  tech160nm_fiaoi012aa1n05x5   g160(.a(new_n245), .b(new_n237), .c(new_n246), .o1(new_n256));
  inv040aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  oab012aa1n06x5               g162(.a(new_n257), .b(new_n234), .c(new_n255), .out0(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n254), .c(new_n210), .d(new_n200), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n258), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[23] ), .b(\b[22] ), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n201), .d(new_n253), .o1(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .out0(new_n266));
  aoi012aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n262), .o1(new_n267));
  xorc02aa1n12x5               g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  aoi112aa1n03x4               g173(.a(new_n261), .b(new_n268), .c(new_n259), .d(new_n264), .o1(new_n269));
  nor002aa1n02x5               g174(.a(new_n267), .b(new_n269), .o1(\s[24] ));
  nand03aa1n02x5               g175(.a(new_n252), .b(new_n264), .c(new_n268), .o1(new_n271));
  nano22aa1n02x4               g176(.a(new_n271), .b(new_n211), .c(new_n231), .out0(new_n272));
  inv000aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  oaoi03aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .o1(new_n274));
  aoi013aa1n06x4               g179(.a(new_n274), .b(new_n257), .c(new_n264), .d(new_n268), .o1(new_n275));
  tech160nm_fioai012aa1n05x5   g180(.a(new_n275), .b(new_n234), .c(new_n271), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n273), .c(new_n210), .d(new_n200), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n276), .c(new_n201), .d(new_n272), .o1(new_n283));
  xorc02aa1n12x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  aoi012aa1n02x7               g190(.a(new_n285), .b(new_n283), .c(new_n281), .o1(new_n286));
  aoi112aa1n03x4               g191(.a(new_n280), .b(new_n284), .c(new_n278), .d(new_n282), .o1(new_n287));
  nor042aa1n03x5               g192(.a(new_n286), .b(new_n287), .o1(\s[26] ));
  oabi12aa1n02x7               g193(.a(new_n199), .b(new_n161), .c(new_n208), .out0(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[22] ), .b(\a[23] ), .out0(new_n290));
  nor043aa1n03x5               g195(.a(new_n255), .b(new_n290), .c(new_n266), .o1(new_n291));
  and002aa1n06x5               g196(.a(new_n284), .b(new_n282), .o(new_n292));
  nano22aa1n12x5               g197(.a(new_n232), .b(new_n291), .c(new_n292), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n289), .c(new_n126), .d(new_n209), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(\b[25] ), .b(\a[26] ), .o1(new_n295));
  oai022aa1n02x5               g200(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n296));
  aoi022aa1n06x5               g201(.a(new_n276), .b(new_n292), .c(new_n295), .d(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n297), .out0(\s[27] ));
  nor042aa1n03x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  inv040aa1n03x5               g205(.a(new_n300), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n291), .b(new_n233), .c(new_n231), .d(new_n222), .o1(new_n302));
  inv000aa1n02x5               g207(.a(new_n292), .o1(new_n303));
  nanp02aa1n02x5               g208(.a(new_n296), .b(new_n295), .o1(new_n304));
  aoai13aa1n04x5               g209(.a(new_n304), .b(new_n303), .c(new_n302), .d(new_n275), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n298), .b(new_n305), .c(new_n201), .d(new_n293), .o1(new_n306));
  xnrc02aa1n12x5               g211(.a(\b[27] ), .b(\a[28] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n306), .c(new_n301), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n298), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n309), .b(new_n294), .c(new_n297), .o1(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n301), .c(new_n307), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n308), .b(new_n311), .o1(\s[28] ));
  norb02aa1n06x5               g217(.a(new_n298), .b(new_n307), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n305), .c(new_n201), .d(new_n293), .o1(new_n314));
  oao003aa1n03x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n315));
  xnrc02aa1n12x5               g220(.a(\b[28] ), .b(\a[29] ), .out0(new_n316));
  aoi012aa1n02x5               g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n313), .o1(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n318), .b(new_n294), .c(new_n297), .o1(new_n319));
  nano22aa1n03x5               g224(.a(new_n319), .b(new_n315), .c(new_n316), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n317), .b(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g227(.a(new_n298), .b(new_n316), .c(new_n307), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n305), .c(new_n201), .d(new_n293), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .out0(new_n326));
  aoi012aa1n02x5               g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n323), .o1(new_n328));
  tech160nm_fiaoi012aa1n02p5x5 g233(.a(new_n328), .b(new_n294), .c(new_n297), .o1(new_n329));
  nano22aa1n03x5               g234(.a(new_n329), .b(new_n325), .c(new_n326), .out0(new_n330));
  norp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[30] ));
  nona32aa1n02x4               g236(.a(new_n298), .b(new_n326), .c(new_n316), .d(new_n307), .out0(new_n332));
  inv000aa1n02x5               g237(.a(new_n332), .o1(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n305), .c(new_n201), .d(new_n293), .o1(new_n334));
  oao003aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n335));
  xnrc02aa1n02x5               g240(.a(\b[30] ), .b(\a[31] ), .out0(new_n336));
  aoi012aa1n02x5               g241(.a(new_n336), .b(new_n334), .c(new_n335), .o1(new_n337));
  tech160nm_fiaoi012aa1n02p5x5 g242(.a(new_n332), .b(new_n294), .c(new_n297), .o1(new_n338));
  nano22aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n336), .out0(new_n339));
  norp02aa1n03x5               g244(.a(new_n337), .b(new_n339), .o1(\s[31] ));
  xnbna2aa1n03x5               g245(.a(new_n143), .b(new_n148), .c(new_n108), .out0(\s[3] ));
  oaoi03aa1n02x5               g246(.a(\a[3] ), .b(\b[2] ), .c(new_n143), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g248(.a(new_n168), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g249(.a(new_n118), .b(new_n168), .c(new_n119), .o1(new_n345));
  xnrb03aa1n02x5               g250(.a(new_n345), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g251(.a(new_n122), .b(new_n153), .c(new_n105), .d(new_n110), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g253(.a(new_n113), .b(new_n347), .c(new_n114), .o1(new_n349));
  xnrb03aa1n02x5               g254(.a(new_n349), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g255(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


