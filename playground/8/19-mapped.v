// Benchmark "adder" written by ABC on Wed Jul 17 16:11:19 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n331,
    new_n334, new_n336, new_n338, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n08x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand42aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1d18x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor022aa1n16x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanb02aa1n12x5               g007(.a(new_n101), .b(new_n102), .out0(new_n103));
  inv040aa1d32x5               g008(.a(\a[3] ), .o1(new_n104));
  inv040aa1d32x5               g009(.a(\b[2] ), .o1(new_n105));
  nand22aa1n12x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  nand02aa1d06x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand22aa1n03x5               g012(.a(new_n106), .b(new_n107), .o1(new_n108));
  aoai13aa1n04x5               g013(.a(new_n102), .b(new_n101), .c(new_n104), .d(new_n105), .o1(new_n109));
  oai013aa1d12x5               g014(.a(new_n109), .b(new_n100), .c(new_n103), .d(new_n108), .o1(new_n110));
  xorc02aa1n06x5               g015(.a(\a[6] ), .b(\b[5] ), .out0(new_n111));
  nor022aa1n16x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1d08x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norb02aa1n06x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  xorc02aa1n03x5               g019(.a(\a[7] ), .b(\b[6] ), .out0(new_n115));
  nand42aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  tech160nm_fixorc02aa1n05x5   g021(.a(\a[5] ), .b(\b[4] ), .out0(new_n117));
  nano22aa1n12x5               g022(.a(new_n116), .b(new_n111), .c(new_n117), .out0(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  nand02aa1n04x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  inv020aa1n04x5               g025(.a(new_n120), .o1(new_n121));
  nor042aa1n04x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oab012aa1n03x5               g027(.a(new_n122), .b(\a[6] ), .c(\b[5] ), .out0(new_n123));
  nona23aa1n03x5               g028(.a(new_n115), .b(new_n114), .c(new_n123), .d(new_n121), .out0(new_n124));
  nona22aa1n06x5               g029(.a(new_n124), .b(new_n119), .c(new_n112), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n125), .b(new_n118), .c(new_n110), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  aoi112aa1n09x5               g034(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n130));
  tech160nm_fixnrc02aa1n04x5   g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  tech160nm_fixnrc02aa1n04x5   g036(.a(\b[9] ), .b(\a[10] ), .out0(new_n132));
  nor042aa1n02x5               g037(.a(new_n132), .b(new_n131), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n134));
  nona22aa1n02x4               g039(.a(new_n134), .b(new_n130), .c(new_n129), .out0(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n09x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand02aa1d12x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n137), .b(new_n135), .c(new_n138), .o1(new_n139));
  nor042aa1n12x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1d28x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n12x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnrc02aa1n02x5               g047(.a(new_n139), .b(new_n142), .out0(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n137), .b(new_n140), .c(new_n141), .d(new_n138), .out0(new_n144));
  norb03aa1n02x5               g049(.a(new_n144), .b(new_n131), .c(new_n132), .out0(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n146));
  aoi112aa1n09x5               g051(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n147));
  norb02aa1n12x5               g052(.a(new_n138), .b(new_n137), .out0(new_n148));
  oai112aa1n06x5               g053(.a(new_n142), .b(new_n148), .c(new_n130), .d(new_n129), .o1(new_n149));
  nona22aa1d18x5               g054(.a(new_n149), .b(new_n147), .c(new_n140), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n146), .b(new_n151), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1n20x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n154), .b(new_n152), .c(new_n155), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n12x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand02aa1d28x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n159), .b(new_n155), .c(new_n154), .d(new_n158), .out0(new_n160));
  tech160nm_fiaoi012aa1n03p5x5 g065(.a(new_n158), .b(new_n154), .c(new_n159), .o1(new_n161));
  aoai13aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n146), .d(new_n151), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  xorc02aa1n12x5               g070(.a(\a[16] ), .b(\b[15] ), .out0(new_n166));
  aoi112aa1n02x7               g071(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n168));
  norb02aa1n03x4               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  nano23aa1n06x5               g074(.a(new_n154), .b(new_n158), .c(new_n159), .d(new_n155), .out0(new_n170));
  nand23aa1n06x5               g075(.a(new_n170), .b(new_n165), .c(new_n166), .o1(new_n171));
  nano22aa1d15x5               g076(.a(new_n171), .b(new_n133), .c(new_n144), .out0(new_n172));
  aoai13aa1n12x5               g077(.a(new_n172), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n173));
  tech160nm_fixnrc02aa1n02p5x5 g078(.a(\b[14] ), .b(\a[15] ), .out0(new_n174));
  tech160nm_fixnrc02aa1n02p5x5 g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  nor043aa1n02x5               g080(.a(new_n160), .b(new_n175), .c(new_n174), .o1(new_n176));
  aoi112aa1n03x5               g081(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n177));
  norp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  inv000aa1n02x5               g083(.a(new_n178), .o1(new_n179));
  oai013aa1n03x5               g084(.a(new_n179), .b(new_n174), .c(new_n175), .d(new_n161), .o1(new_n180));
  aoi112aa1n09x5               g085(.a(new_n180), .b(new_n177), .c(new_n150), .d(new_n176), .o1(new_n181));
  xorc02aa1n12x5               g086(.a(\a[17] ), .b(\b[16] ), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n173), .c(new_n181), .out0(\s[17] ));
  orn002aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .o(new_n184));
  nor043aa1n02x5               g089(.a(new_n100), .b(new_n103), .c(new_n108), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n109), .b(new_n185), .out0(new_n186));
  nanb02aa1n03x5               g091(.a(new_n112), .b(new_n113), .out0(new_n187));
  xnrc02aa1n12x5               g092(.a(\b[6] ), .b(\a[7] ), .out0(new_n188));
  nona23aa1n02x4               g093(.a(new_n111), .b(new_n117), .c(new_n188), .d(new_n187), .out0(new_n189));
  oai022aa1n02x5               g094(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n190));
  nano23aa1n02x4               g095(.a(new_n188), .b(new_n187), .c(new_n190), .d(new_n120), .out0(new_n191));
  norp03aa1n02x5               g096(.a(new_n191), .b(new_n119), .c(new_n112), .o1(new_n192));
  oai012aa1n03x5               g097(.a(new_n192), .b(new_n186), .c(new_n189), .o1(new_n193));
  nand42aa1n02x5               g098(.a(new_n150), .b(new_n176), .o1(new_n194));
  nona22aa1n03x5               g099(.a(new_n194), .b(new_n180), .c(new_n177), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n182), .b(new_n195), .c(new_n193), .d(new_n172), .o1(new_n196));
  tech160nm_fixorc02aa1n03p5x5 g101(.a(\a[18] ), .b(\b[17] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n196), .c(new_n184), .out0(\s[18] ));
  and002aa1n02x7               g103(.a(new_n197), .b(new_n182), .o(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nor042aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  aoi112aa1n09x5               g106(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n202));
  norp02aa1n02x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n200), .c(new_n173), .d(new_n181), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand22aa1n03x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor042aa1d18x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1d24x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n15x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoi112aa1n02x7               g116(.a(new_n207), .b(new_n211), .c(new_n204), .d(new_n208), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n207), .o1(new_n213));
  norb02aa1n09x5               g118(.a(new_n208), .b(new_n207), .out0(new_n214));
  nand42aa1n02x5               g119(.a(new_n204), .b(new_n214), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n211), .o1(new_n216));
  aoi012aa1n03x5               g121(.a(new_n216), .b(new_n215), .c(new_n213), .o1(new_n217));
  norp02aa1n03x5               g122(.a(new_n217), .b(new_n212), .o1(\s[20] ));
  nano23aa1n06x5               g123(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n219));
  nand03aa1n02x5               g124(.a(new_n219), .b(new_n182), .c(new_n197), .o1(new_n220));
  aoi112aa1n09x5               g125(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n221));
  oai112aa1n06x5               g126(.a(new_n214), .b(new_n211), .c(new_n202), .d(new_n201), .o1(new_n222));
  nona22aa1d18x5               g127(.a(new_n222), .b(new_n221), .c(new_n209), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n220), .c(new_n173), .d(new_n181), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nand42aa1n03x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  nor022aa1n08x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  nand02aa1d04x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  norb02aa1n12x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  aoi112aa1n02x7               g137(.a(new_n227), .b(new_n232), .c(new_n225), .d(new_n229), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n227), .o1(new_n234));
  tech160nm_finand02aa1n05x5   g139(.a(new_n225), .b(new_n229), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n232), .o1(new_n236));
  aoi012aa1n03x5               g141(.a(new_n236), .b(new_n235), .c(new_n234), .o1(new_n237));
  norp02aa1n03x5               g142(.a(new_n237), .b(new_n233), .o1(\s[22] ));
  nano23aa1n06x5               g143(.a(new_n227), .b(new_n230), .c(new_n231), .d(new_n228), .out0(new_n239));
  nand23aa1n03x5               g144(.a(new_n199), .b(new_n219), .c(new_n239), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n230), .b(new_n227), .c(new_n231), .o1(new_n241));
  aobi12aa1n06x5               g146(.a(new_n241), .b(new_n223), .c(new_n239), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n240), .c(new_n173), .d(new_n181), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n16x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  nand02aa1d04x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  nanb02aa1n12x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  nor002aa1n16x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  nand22aa1n04x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  nanb02aa1n09x5               g155(.a(new_n249), .b(new_n250), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  aoi112aa1n02x7               g157(.a(new_n245), .b(new_n252), .c(new_n243), .d(new_n248), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n245), .o1(new_n254));
  tech160nm_finand02aa1n05x5   g159(.a(new_n243), .b(new_n248), .o1(new_n255));
  aoi012aa1n03x5               g160(.a(new_n251), .b(new_n255), .c(new_n254), .o1(new_n256));
  norp02aa1n03x5               g161(.a(new_n256), .b(new_n253), .o1(\s[24] ));
  nona23aa1n12x5               g162(.a(new_n250), .b(new_n246), .c(new_n245), .d(new_n249), .out0(new_n258));
  inv040aa1n03x5               g163(.a(new_n258), .o1(new_n259));
  nanb03aa1n02x5               g164(.a(new_n220), .b(new_n259), .c(new_n239), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n249), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(new_n245), .b(new_n250), .o1(new_n262));
  oai112aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n258), .d(new_n241), .o1(new_n263));
  nano22aa1n03x7               g168(.a(new_n258), .b(new_n229), .c(new_n232), .out0(new_n264));
  tech160nm_fiaoi012aa1n03p5x5 g169(.a(new_n263), .b(new_n223), .c(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n260), .c(new_n173), .d(new_n181), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  tech160nm_fixorc02aa1n03p5x5 g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xorc02aa1n12x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  aoi112aa1n02x7               g175(.a(new_n268), .b(new_n270), .c(new_n266), .d(new_n269), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n268), .o1(new_n272));
  nand42aa1n04x5               g177(.a(new_n266), .b(new_n269), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n270), .o1(new_n274));
  aoi012aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n272), .o1(new_n275));
  norp02aa1n03x5               g180(.a(new_n275), .b(new_n271), .o1(\s[26] ));
  and002aa1n06x5               g181(.a(new_n270), .b(new_n269), .o(new_n277));
  nano32aa1n03x7               g182(.a(new_n220), .b(new_n277), .c(new_n239), .d(new_n259), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n195), .c(new_n193), .d(new_n172), .o1(new_n279));
  inv000aa1d42x5               g184(.a(\a[26] ), .o1(new_n280));
  nanb02aa1n02x5               g185(.a(\b[25] ), .b(new_n280), .out0(new_n281));
  aob012aa1n12x5               g186(.a(new_n268), .b(\b[25] ), .c(\a[26] ), .out0(new_n282));
  nor043aa1n03x5               g187(.a(new_n241), .b(new_n247), .c(new_n251), .o1(new_n283));
  nano22aa1n03x7               g188(.a(new_n283), .b(new_n261), .c(new_n262), .out0(new_n284));
  nand22aa1n03x5               g189(.a(new_n223), .b(new_n264), .o1(new_n285));
  aobi12aa1n06x5               g190(.a(new_n277), .b(new_n285), .c(new_n284), .out0(new_n286));
  nano22aa1n03x7               g191(.a(new_n286), .b(new_n281), .c(new_n282), .out0(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n279), .c(new_n287), .out0(\s[27] ));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  inv040aa1n03x5               g195(.a(new_n290), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n288), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n279), .c(new_n287), .o1(new_n293));
  xnrc02aa1n12x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n293), .b(new_n291), .c(new_n294), .out0(new_n295));
  nand22aa1n03x5               g200(.a(new_n173), .b(new_n181), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n277), .b(new_n263), .c(new_n223), .d(new_n264), .o1(new_n297));
  nand03aa1n03x5               g202(.a(new_n297), .b(new_n281), .c(new_n282), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n288), .b(new_n298), .c(new_n296), .d(new_n278), .o1(new_n299));
  aoi012aa1n02x7               g204(.a(new_n294), .b(new_n299), .c(new_n291), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n295), .o1(\s[28] ));
  norb02aa1n02x5               g206(.a(new_n288), .b(new_n294), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n296), .d(new_n278), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .out0(new_n305));
  aoi012aa1n03x5               g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n302), .o1(new_n307));
  tech160nm_fiaoi012aa1n05x5   g212(.a(new_n307), .b(new_n279), .c(new_n287), .o1(new_n308));
  nano22aa1n02x5               g213(.a(new_n308), .b(new_n304), .c(new_n305), .out0(new_n309));
  nor002aa1n02x5               g214(.a(new_n306), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g216(.a(new_n288), .b(new_n305), .c(new_n294), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n298), .c(new_n296), .d(new_n278), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n304), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n312), .o1(new_n317));
  tech160nm_fiaoi012aa1n02p5x5 g222(.a(new_n317), .b(new_n279), .c(new_n287), .o1(new_n318));
  nano22aa1n02x4               g223(.a(new_n318), .b(new_n314), .c(new_n315), .out0(new_n319));
  nor002aa1n02x5               g224(.a(new_n316), .b(new_n319), .o1(\s[30] ));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  norb02aa1n03x5               g226(.a(new_n312), .b(new_n315), .out0(new_n322));
  inv020aa1n02x5               g227(.a(new_n322), .o1(new_n323));
  aoi012aa1n02x5               g228(.a(new_n323), .b(new_n279), .c(new_n287), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n325));
  nano22aa1n03x5               g230(.a(new_n324), .b(new_n321), .c(new_n325), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n322), .b(new_n298), .c(new_n296), .d(new_n278), .o1(new_n327));
  aoi012aa1n03x5               g232(.a(new_n321), .b(new_n327), .c(new_n325), .o1(new_n328));
  nor002aa1n02x5               g233(.a(new_n328), .b(new_n326), .o1(\s[31] ));
  xnbna2aa1n03x5               g234(.a(new_n100), .b(new_n107), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g235(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g237(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g238(.a(new_n122), .b(new_n110), .c(new_n117), .o1(new_n334));
  xnrc02aa1n02x5               g239(.a(new_n334), .b(new_n111), .out0(\s[6] ));
  nanp02aa1n02x5               g240(.a(new_n334), .b(new_n111), .o1(new_n336));
  xnbna2aa1n03x5               g241(.a(new_n188), .b(new_n336), .c(new_n120), .out0(\s[7] ));
  orn002aa1n02x5               g242(.a(\a[7] ), .b(\b[6] ), .o(new_n338));
  nona22aa1n02x4               g243(.a(new_n336), .b(new_n188), .c(new_n121), .out0(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n114), .b(new_n339), .c(new_n338), .out0(\s[8] ));
  xnrb03aa1n02x5               g245(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


