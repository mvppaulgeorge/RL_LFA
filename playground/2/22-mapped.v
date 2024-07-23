// Benchmark "adder" written by ABC on Wed Jul 17 13:07:24 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n304, new_n307, new_n308, new_n310, new_n312, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1d32x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand02aa1d06x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  tech160nm_finor002aa1n05x5   g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanb02aa1n02x5               g006(.a(new_n100), .b(new_n101), .out0(new_n102));
  nor022aa1n04x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanb02aa1n02x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  tech160nm_fixorc02aa1n04x5   g010(.a(\a[6] ), .b(\b[5] ), .out0(new_n106));
  xorc02aa1n06x5               g011(.a(\a[5] ), .b(\b[4] ), .out0(new_n107));
  nona23aa1n09x5               g012(.a(new_n106), .b(new_n107), .c(new_n105), .d(new_n102), .out0(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nand42aa1n04x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nor002aa1n20x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand02aa1d24x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nano22aa1n02x4               g017(.a(new_n111), .b(new_n110), .c(new_n112), .out0(new_n113));
  nand22aa1n03x5               g018(.a(\b[0] ), .b(\a[1] ), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nona22aa1n02x4               g020(.a(new_n110), .b(new_n115), .c(new_n114), .out0(new_n116));
  oab012aa1n12x5               g021(.a(new_n111), .b(\a[4] ), .c(\b[3] ), .out0(new_n117));
  inv000aa1d42x5               g022(.a(new_n117), .o1(new_n118));
  aoai13aa1n04x5               g023(.a(new_n109), .b(new_n118), .c(new_n113), .d(new_n116), .o1(new_n119));
  nona23aa1n12x5               g024(.a(new_n104), .b(new_n101), .c(new_n100), .d(new_n103), .out0(new_n120));
  norp02aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nor002aa1d32x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n122), .b(new_n123), .c(new_n121), .o1(new_n124));
  oa0012aa1n03x5               g029(.a(new_n101), .b(new_n103), .c(new_n100), .o(new_n125));
  oab012aa1n06x5               g030(.a(new_n125), .b(new_n120), .c(new_n124), .out0(new_n126));
  oaih12aa1n06x5               g031(.a(new_n126), .b(new_n119), .c(new_n108), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n127), .d(new_n99), .o1(new_n128));
  nano22aa1n03x7               g033(.a(new_n120), .b(new_n106), .c(new_n107), .out0(new_n129));
  norb02aa1n12x5               g034(.a(new_n112), .b(new_n111), .out0(new_n130));
  oai112aa1n04x5               g035(.a(new_n130), .b(new_n110), .c(new_n114), .d(new_n115), .o1(new_n131));
  aoi022aa1n12x5               g036(.a(new_n131), .b(new_n117), .c(\b[3] ), .d(\a[4] ), .o1(new_n132));
  oabi12aa1n06x5               g037(.a(new_n125), .b(new_n120), .c(new_n124), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n99), .b(new_n133), .c(new_n132), .d(new_n129), .o1(new_n134));
  nona22aa1n02x4               g039(.a(new_n134), .b(new_n98), .c(new_n97), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n128), .b(new_n135), .o1(\s[10] ));
  nanp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norp02aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  xobna2aa1n03x5               g045(.a(new_n140), .b(new_n135), .c(new_n137), .out0(\s[11] ));
  aoi013aa1n02x4               g046(.a(new_n139), .b(new_n135), .c(new_n137), .d(new_n140), .o1(new_n142));
  nor042aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1d28x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nona22aa1n03x5               g050(.a(new_n144), .b(new_n143), .c(new_n139), .out0(new_n146));
  aoi013aa1n02x4               g051(.a(new_n146), .b(new_n135), .c(new_n140), .d(new_n137), .o1(new_n147));
  oabi12aa1n02x5               g052(.a(new_n147), .b(new_n142), .c(new_n145), .out0(\s[12] ));
  nano23aa1d15x5               g053(.a(new_n143), .b(new_n139), .c(new_n144), .d(new_n138), .out0(new_n149));
  nanb02aa1n12x5               g054(.a(new_n98), .b(new_n99), .out0(new_n150));
  nona22aa1d24x5               g055(.a(new_n149), .b(new_n97), .c(new_n150), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n133), .c(new_n132), .d(new_n129), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n98), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(\a[10] ), .b(\b[9] ), .c(new_n154), .o1(new_n155));
  aoi022aa1n06x5               g060(.a(new_n149), .b(new_n155), .c(new_n146), .d(new_n144), .o1(new_n156));
  xnrc02aa1n12x5               g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n153), .c(new_n156), .out0(\s[13] ));
  orn002aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .o(new_n159));
  ao0012aa1n03x7               g064(.a(new_n157), .b(new_n153), .c(new_n156), .o(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n159), .out0(\s[14] ));
  nor042aa1n09x5               g067(.a(new_n161), .b(new_n157), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  norp02aa1n03x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  tech160nm_fioai012aa1n04x5   g072(.a(new_n167), .b(new_n166), .c(new_n165), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n164), .c(new_n153), .d(new_n156), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  xnrc02aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  norp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  inv030aa1d32x5               g078(.a(\a[16] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[15] ), .o1(new_n175));
  nanp02aa1n03x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand02aa1n04x5               g082(.a(new_n176), .b(new_n177), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n173), .c(new_n169), .d(new_n172), .o1(new_n179));
  oai112aa1n02x5               g084(.a(new_n176), .b(new_n177), .c(\b[14] ), .d(\a[15] ), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n172), .d(new_n169), .o1(\s[16] ));
  nor042aa1n04x5               g086(.a(new_n171), .b(new_n178), .o1(new_n182));
  nano22aa1d15x5               g087(.a(new_n151), .b(new_n163), .c(new_n182), .out0(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n133), .c(new_n132), .d(new_n129), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n163), .b(new_n182), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n174), .b(new_n175), .c(new_n173), .o1(new_n186));
  oai013aa1n06x5               g091(.a(new_n186), .b(new_n171), .c(new_n168), .d(new_n178), .o1(new_n187));
  oab012aa1d15x5               g092(.a(new_n187), .b(new_n156), .c(new_n185), .out0(new_n188));
  xorc02aa1n02x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n184), .c(new_n188), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  nanp02aa1n06x5               g096(.a(new_n184), .b(new_n188), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  tech160nm_fiaoi012aa1n05x5   g098(.a(new_n193), .b(new_n192), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  oabi12aa1n03x5               g100(.a(new_n187), .b(new_n156), .c(new_n185), .out0(new_n196));
  inv000aa1d42x5               g101(.a(\a[17] ), .o1(new_n197));
  xroi22aa1d04x5               g102(.a(new_n197), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n196), .c(new_n127), .d(new_n183), .o1(new_n199));
  nor042aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  aoi112aa1n09x5               g105(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  nor002aa1n20x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1d10x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n12x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x5               g112(.a(new_n205), .b(new_n199), .c(new_n202), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n199), .b(new_n202), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n203), .b(new_n209), .c(new_n204), .o1(new_n210));
  nor042aa1n12x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand02aa1d20x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n12x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  nona22aa1n02x4               g118(.a(new_n212), .b(new_n211), .c(new_n203), .out0(new_n214));
  oai022aa1n02x5               g119(.a(new_n210), .b(new_n213), .c(new_n214), .d(new_n208), .o1(\s[20] ));
  nano23aa1n09x5               g120(.a(new_n203), .b(new_n211), .c(new_n212), .d(new_n204), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n198), .b(new_n216), .o1(new_n217));
  aboi22aa1n03x5               g122(.a(new_n202), .b(new_n216), .c(new_n212), .d(new_n214), .out0(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n217), .c(new_n184), .d(new_n188), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  xorc02aa1n12x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  nor042aa1n06x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n222), .c(new_n219), .d(new_n221), .o1(new_n224));
  oabi12aa1n02x5               g129(.a(new_n223), .b(\a[21] ), .c(\b[20] ), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n224), .b(new_n225), .c(new_n221), .d(new_n219), .o1(\s[22] ));
  norb02aa1n02x5               g131(.a(new_n221), .b(new_n223), .out0(new_n227));
  nand23aa1n04x5               g132(.a(new_n227), .b(new_n198), .c(new_n216), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n205), .b(new_n213), .c(new_n201), .d(new_n200), .o1(new_n229));
  tech160nm_fioai012aa1n03p5x5 g134(.a(new_n212), .b(new_n211), .c(new_n203), .o1(new_n230));
  nanb02aa1n03x5               g135(.a(new_n223), .b(new_n221), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\a[22] ), .o1(new_n232));
  inv040aa1d32x5               g137(.a(\b[21] ), .o1(new_n233));
  oao003aa1n09x5               g138(.a(new_n232), .b(new_n233), .c(new_n222), .carry(new_n234));
  inv040aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n12x5               g140(.a(new_n235), .b(new_n231), .c(new_n229), .d(new_n230), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n04x5               g142(.a(new_n237), .b(new_n228), .c(new_n184), .d(new_n188), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xnrc02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n243));
  norp02aa1n02x5               g148(.a(new_n242), .b(new_n240), .o1(new_n244));
  aob012aa1n03x5               g149(.a(new_n244), .b(new_n238), .c(new_n241), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n243), .b(new_n245), .o1(\s[24] ));
  norb02aa1n03x5               g151(.a(new_n241), .b(new_n242), .out0(new_n247));
  nanb03aa1n02x5               g152(.a(new_n217), .b(new_n247), .c(new_n227), .out0(new_n248));
  aoi012aa1n02x5               g153(.a(new_n244), .b(\a[24] ), .c(\b[23] ), .o1(new_n249));
  tech160nm_fiaoi012aa1n02p5x5 g154(.a(new_n249), .b(new_n236), .c(new_n247), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n248), .c(new_n184), .d(new_n188), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  xorc02aa1n02x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xnrc02aa1n02x5               g159(.a(\b[25] ), .b(\a[26] ), .out0(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n254), .c(new_n251), .d(new_n253), .o1(new_n256));
  oabi12aa1n02x5               g161(.a(new_n255), .b(\a[25] ), .c(\b[24] ), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n256), .b(new_n257), .c(new_n253), .d(new_n251), .o1(\s[26] ));
  norb02aa1n03x5               g163(.a(new_n253), .b(new_n255), .out0(new_n259));
  nano22aa1n09x5               g164(.a(new_n228), .b(new_n247), .c(new_n259), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n196), .c(new_n127), .d(new_n183), .o1(new_n261));
  aoai13aa1n09x5               g166(.a(new_n259), .b(new_n249), .c(new_n236), .d(new_n247), .o1(new_n262));
  aob012aa1n02x5               g167(.a(new_n257), .b(\b[25] ), .c(\a[26] ), .out0(new_n263));
  nanp03aa1n03x5               g168(.a(new_n261), .b(new_n262), .c(new_n263), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  inv000aa1n02x5               g170(.a(new_n260), .o1(new_n266));
  aoi012aa1n12x5               g171(.a(new_n266), .b(new_n184), .c(new_n188), .o1(new_n267));
  nand22aa1n04x5               g172(.a(new_n262), .b(new_n263), .o1(new_n268));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  oaoi13aa1n03x5               g175(.a(new_n269), .b(new_n270), .c(new_n268), .d(new_n267), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[28] ), .b(\b[27] ), .out0(new_n272));
  oaih12aa1n02x5               g177(.a(new_n270), .b(new_n268), .c(new_n267), .o1(new_n273));
  oai112aa1n02x7               g178(.a(new_n273), .b(new_n272), .c(\b[26] ), .d(\a[27] ), .o1(new_n274));
  oaih12aa1n02x5               g179(.a(new_n274), .b(new_n271), .c(new_n272), .o1(\s[28] ));
  xorc02aa1n02x5               g180(.a(\a[29] ), .b(\b[28] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  and002aa1n02x5               g182(.a(new_n272), .b(new_n270), .o(new_n278));
  orn002aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .o(new_n279));
  oaoi03aa1n02x5               g184(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n277), .b(new_n280), .c(new_n264), .d(new_n278), .o1(new_n281));
  oai012aa1n03x5               g186(.a(new_n278), .b(new_n268), .c(new_n267), .o1(new_n282));
  nona22aa1n03x5               g187(.a(new_n282), .b(new_n280), .c(new_n277), .out0(new_n283));
  nanp02aa1n03x5               g188(.a(new_n281), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n114), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g190(.a(new_n277), .b(new_n270), .c(new_n272), .out0(new_n286));
  inv000aa1d42x5               g191(.a(\b[28] ), .o1(new_n287));
  oaib12aa1n02x5               g192(.a(new_n280), .b(new_n287), .c(\a[29] ), .out0(new_n288));
  oaib12aa1n02x5               g193(.a(new_n288), .b(\a[29] ), .c(new_n287), .out0(new_n289));
  oaoi13aa1n03x5               g194(.a(new_n289), .b(new_n286), .c(new_n268), .d(new_n267), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[30] ), .b(\b[29] ), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n286), .b(new_n268), .c(new_n267), .o1(new_n292));
  norb02aa1n02x5               g197(.a(new_n291), .b(new_n289), .out0(new_n293));
  nand42aa1n02x5               g198(.a(new_n292), .b(new_n293), .o1(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n291), .o1(\s[30] ));
  nano32aa1n02x4               g200(.a(new_n277), .b(new_n291), .c(new_n270), .d(new_n272), .out0(new_n296));
  aoi012aa1n02x5               g201(.a(new_n293), .b(\a[30] ), .c(\b[29] ), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n264), .d(new_n296), .o1(new_n299));
  oaih12aa1n02x5               g204(.a(new_n296), .b(new_n268), .c(new_n267), .o1(new_n300));
  nona22aa1n03x5               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  nanp02aa1n03x5               g206(.a(new_n299), .b(new_n301), .o1(\s[31] ));
  xobna2aa1n03x5               g207(.a(new_n130), .b(new_n116), .c(new_n110), .out0(\s[3] ));
  aoi012aa1n02x5               g208(.a(new_n111), .b(new_n113), .c(new_n116), .o1(new_n304));
  xnrb03aa1n02x5               g209(.a(new_n304), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrc02aa1n02x5               g210(.a(new_n119), .b(new_n107), .out0(\s[5] ));
  inv000aa1d42x5               g211(.a(new_n123), .o1(new_n307));
  nanp02aa1n02x5               g212(.a(new_n132), .b(new_n107), .o1(new_n308));
  xnbna2aa1n03x5               g213(.a(new_n106), .b(new_n308), .c(new_n307), .out0(\s[6] ));
  nanp03aa1n02x5               g214(.a(new_n308), .b(new_n106), .c(new_n307), .o1(new_n310));
  xnbna2aa1n03x5               g215(.a(new_n105), .b(new_n310), .c(new_n122), .out0(\s[7] ));
  orn002aa1n02x5               g216(.a(\a[7] ), .b(\b[6] ), .o(new_n312));
  nanb03aa1n02x5               g217(.a(new_n105), .b(new_n310), .c(new_n122), .out0(new_n313));
  xobna2aa1n03x5               g218(.a(new_n102), .b(new_n313), .c(new_n312), .out0(\s[8] ));
  xorb03aa1n02x5               g219(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


