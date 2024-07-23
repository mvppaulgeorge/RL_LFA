// Benchmark "adder" written by ABC on Thu Jul 18 09:02:33 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  nanp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[8] ), .o1(new_n99));
  nanb02aa1n12x5               g004(.a(\a[9] ), .b(new_n99), .out0(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[7] ), .b(\a[8] ), .out0(new_n101));
  tech160nm_fixnrc02aa1n05x5   g006(.a(\b[6] ), .b(\a[7] ), .out0(new_n102));
  oai022aa1n04x7               g007(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n103));
  aob012aa1n06x5               g008(.a(new_n103), .b(\b[5] ), .c(\a[6] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[8] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[7] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  oaoi03aa1n02x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  oai013aa1d12x5               g013(.a(new_n108), .b(new_n104), .c(new_n101), .d(new_n102), .o1(new_n109));
  inv040aa1n02x5               g014(.a(new_n109), .o1(new_n110));
  norp02aa1n02x5               g015(.a(new_n102), .b(new_n101), .o1(new_n111));
  nor002aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nor002aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  nand22aa1n06x5               g023(.a(\b[0] ), .b(\a[1] ), .o1(new_n119));
  tech160nm_fioai012aa1n05x5   g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  inv000aa1n02x5               g025(.a(new_n114), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[4] ), .b(\b[3] ), .c(new_n121), .o1(new_n122));
  oabi12aa1n03x5               g027(.a(new_n122), .b(new_n116), .c(new_n120), .out0(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .out0(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .out0(new_n125));
  norp02aa1n02x5               g030(.a(new_n125), .b(new_n124), .o1(new_n126));
  nanp03aa1n03x5               g031(.a(new_n123), .b(new_n111), .c(new_n126), .o1(new_n127));
  nanp03aa1n03x5               g032(.a(new_n127), .b(new_n100), .c(new_n110), .o1(new_n128));
  xobna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  and002aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o(new_n130));
  nanp02aa1n03x5               g035(.a(new_n128), .b(new_n98), .o1(new_n131));
  xnrc02aa1n03x5               g036(.a(\b[10] ), .b(\a[11] ), .out0(new_n132));
  aoi112aa1n03x5               g037(.a(new_n132), .b(new_n130), .c(new_n131), .d(new_n97), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n132), .b(new_n130), .c(new_n131), .d(new_n97), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(\s[11] ));
  norp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  tech160nm_fixnrc02aa1n02p5x5 g041(.a(\b[11] ), .b(\a[12] ), .out0(new_n137));
  oai012aa1n02x5               g042(.a(new_n137), .b(new_n133), .c(new_n136), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n131), .b(new_n97), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n132), .c(new_n130), .out0(new_n140));
  nona22aa1n02x4               g045(.a(new_n140), .b(new_n137), .c(new_n136), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n138), .b(new_n141), .o1(\s[12] ));
  xorc02aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .out0(new_n143));
  oaoi03aa1n02x5               g048(.a(\a[10] ), .b(\b[9] ), .c(new_n100), .o1(new_n144));
  nanb03aa1n06x5               g049(.a(new_n137), .b(new_n144), .c(new_n143), .out0(new_n145));
  oai022aa1n02x5               g050(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n146));
  aob012aa1n02x5               g051(.a(new_n146), .b(\b[11] ), .c(\a[12] ), .out0(new_n147));
  and002aa1n02x5               g052(.a(new_n145), .b(new_n147), .o(new_n148));
  nanp02aa1n02x5               g053(.a(new_n100), .b(new_n98), .o1(new_n149));
  nona32aa1n03x5               g054(.a(new_n97), .b(new_n149), .c(new_n137), .d(new_n132), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n148), .b(new_n150), .c(new_n127), .d(new_n110), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d32x5               g057(.a(\a[14] ), .o1(new_n153));
  nor042aa1n09x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  xnrc02aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoi012aa1n03x5               g061(.a(new_n154), .b(new_n151), .c(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  nano23aa1n03x5               g063(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n159));
  inv020aa1n03x5               g064(.a(new_n120), .o1(new_n160));
  tech160nm_fiaoi012aa1n04x5   g065(.a(new_n122), .b(new_n159), .c(new_n160), .o1(new_n161));
  nano22aa1n03x7               g066(.a(new_n161), .b(new_n111), .c(new_n126), .out0(new_n162));
  oabi12aa1n02x5               g067(.a(new_n150), .b(new_n162), .c(new_n109), .out0(new_n163));
  xnrc02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .out0(new_n164));
  nor042aa1n03x5               g069(.a(new_n164), .b(new_n155), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  inv040aa1d32x5               g071(.a(\b[13] ), .o1(new_n167));
  oao003aa1n09x5               g072(.a(new_n153), .b(new_n167), .c(new_n154), .carry(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n04x5               g074(.a(new_n169), .b(new_n166), .c(new_n163), .d(new_n148), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  nor042aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n174), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n174), .b(new_n168), .c(new_n151), .d(new_n165), .o1(new_n179));
  nona22aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n172), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n178), .b(new_n180), .o1(\s[16] ));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  nano23aa1n03x7               g087(.a(new_n172), .b(new_n175), .c(new_n176), .d(new_n173), .out0(new_n183));
  oa0012aa1n02x5               g088(.a(new_n176), .b(new_n175), .c(new_n172), .o(new_n184));
  tech160nm_fiaoi012aa1n03p5x5 g089(.a(new_n184), .b(new_n183), .c(new_n168), .o1(new_n185));
  nona22aa1n03x5               g090(.a(new_n183), .b(new_n164), .c(new_n155), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n185), .b(new_n186), .c(new_n145), .d(new_n147), .o1(new_n187));
  nor042aa1n03x5               g092(.a(new_n150), .b(new_n186), .o1(new_n188));
  oaoi13aa1n12x5               g093(.a(new_n187), .b(new_n188), .c(new_n162), .d(new_n109), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(new_n182), .out0(\s[17] ));
  oaoi03aa1n03x5               g095(.a(\a[17] ), .b(\b[16] ), .c(new_n189), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g097(.a(\a[18] ), .o1(new_n193));
  xroi22aa1d06x4               g098(.a(new_n182), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  oai022aa1n02x7               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(\b[17] ), .out0(new_n197));
  tech160nm_fioai012aa1n05x5   g102(.a(new_n197), .b(new_n189), .c(new_n195), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand02aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nor002aa1n12x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand42aa1n06x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n201), .c(new_n198), .d(new_n203), .o1(new_n207));
  aoi013aa1n03x5               g112(.a(new_n109), .b(new_n123), .c(new_n111), .d(new_n126), .o1(new_n208));
  inv000aa1n02x5               g113(.a(new_n188), .o1(new_n209));
  oabi12aa1n06x5               g114(.a(new_n187), .b(new_n209), .c(new_n208), .out0(new_n210));
  nanb02aa1n02x5               g115(.a(\b[16] ), .b(new_n182), .out0(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n203), .b(new_n212), .c(new_n210), .d(new_n194), .o1(new_n213));
  nona22aa1n02x4               g118(.a(new_n213), .b(new_n206), .c(new_n201), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n207), .b(new_n214), .o1(\s[20] ));
  nona23aa1n09x5               g120(.a(new_n205), .b(new_n202), .c(new_n201), .d(new_n204), .out0(new_n216));
  oa0012aa1n03x5               g121(.a(new_n205), .b(new_n204), .c(new_n201), .o(new_n217));
  oabi12aa1n18x5               g122(.a(new_n217), .b(new_n197), .c(new_n216), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[17] ), .b(\b[16] ), .out0(new_n220));
  xnrc02aa1n02x5               g125(.a(\b[17] ), .b(\a[18] ), .out0(new_n221));
  norb03aa1n06x5               g126(.a(new_n220), .b(new_n216), .c(new_n221), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  tech160nm_fioai012aa1n05x5   g128(.a(new_n219), .b(new_n189), .c(new_n223), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  tech160nm_fixnrc02aa1n05x5   g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n226), .c(new_n224), .d(new_n228), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n228), .b(new_n218), .c(new_n210), .d(new_n222), .o1(new_n231));
  nona22aa1n02x4               g136(.a(new_n231), .b(new_n229), .c(new_n226), .out0(new_n232));
  nanp02aa1n03x5               g137(.a(new_n230), .b(new_n232), .o1(\s[22] ));
  norp02aa1n24x5               g138(.a(new_n229), .b(new_n227), .o1(new_n234));
  inv040aa1d32x5               g139(.a(\a[22] ), .o1(new_n235));
  inv040aa1d32x5               g140(.a(\b[21] ), .o1(new_n236));
  oao003aa1n12x5               g141(.a(new_n235), .b(new_n236), .c(new_n226), .carry(new_n237));
  aoi012aa1d24x5               g142(.a(new_n237), .b(new_n218), .c(new_n234), .o1(new_n238));
  nano23aa1n06x5               g143(.a(new_n201), .b(new_n204), .c(new_n205), .d(new_n202), .out0(new_n239));
  nand23aa1d12x5               g144(.a(new_n194), .b(new_n234), .c(new_n239), .o1(new_n240));
  tech160nm_fioai012aa1n05x5   g145(.a(new_n238), .b(new_n189), .c(new_n240), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  xorc02aa1n12x5               g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[23] ), .b(\a[24] ), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n244), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n238), .o1(new_n247));
  inv000aa1n02x5               g152(.a(new_n240), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n244), .b(new_n247), .c(new_n210), .d(new_n248), .o1(new_n249));
  nona22aa1n02x4               g154(.a(new_n249), .b(new_n245), .c(new_n243), .out0(new_n250));
  nanp02aa1n02x5               g155(.a(new_n246), .b(new_n250), .o1(\s[24] ));
  norb02aa1n06x5               g156(.a(new_n244), .b(new_n245), .out0(new_n252));
  inv040aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  nano32aa1n03x7               g158(.a(new_n253), .b(new_n194), .c(new_n234), .d(new_n239), .out0(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n234), .b(new_n217), .c(new_n239), .d(new_n212), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n237), .o1(new_n257));
  oai022aa1n02x5               g162(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n258));
  aob012aa1n02x5               g163(.a(new_n258), .b(\b[23] ), .c(\a[24] ), .out0(new_n259));
  aoai13aa1n12x5               g164(.a(new_n259), .b(new_n253), .c(new_n256), .d(new_n257), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  tech160nm_fioai012aa1n05x5   g166(.a(new_n261), .b(new_n189), .c(new_n255), .o1(new_n262));
  xorb03aa1n02x5               g167(.a(new_n262), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  xorc02aa1n03x5               g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  tech160nm_fixnrc02aa1n02p5x5 g170(.a(\b[25] ), .b(\a[26] ), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .d(new_n265), .o1(new_n267));
  aoai13aa1n02x5               g172(.a(new_n265), .b(new_n260), .c(new_n210), .d(new_n254), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n268), .b(new_n266), .c(new_n264), .out0(new_n269));
  nanp02aa1n03x5               g174(.a(new_n267), .b(new_n269), .o1(\s[26] ));
  nanp02aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o1(new_n271));
  norb02aa1n06x5               g176(.a(new_n265), .b(new_n266), .out0(new_n272));
  oai022aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n273));
  aoi022aa1n06x5               g178(.a(new_n260), .b(new_n272), .c(new_n271), .d(new_n273), .o1(new_n274));
  nano22aa1d15x5               g179(.a(new_n240), .b(new_n252), .c(new_n272), .out0(new_n275));
  inv020aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  oai012aa1n09x5               g181(.a(new_n274), .b(new_n189), .c(new_n276), .o1(new_n277));
  xorb03aa1n03x5               g182(.a(new_n277), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n279), .c(new_n277), .d(new_n280), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n252), .b(new_n237), .c(new_n218), .d(new_n234), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n272), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n273), .b(new_n271), .o1(new_n285));
  aoai13aa1n04x5               g190(.a(new_n285), .b(new_n284), .c(new_n283), .d(new_n259), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n280), .b(new_n286), .c(new_n210), .d(new_n275), .o1(new_n287));
  nona22aa1n02x4               g192(.a(new_n287), .b(new_n281), .c(new_n279), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n282), .b(new_n288), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n280), .b(new_n281), .out0(new_n290));
  aoai13aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n210), .d(new_n275), .o1(new_n291));
  inv000aa1n03x5               g196(.a(new_n279), .o1(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nona22aa1n02x4               g199(.a(new_n291), .b(new_n293), .c(new_n294), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n294), .b(new_n293), .c(new_n277), .d(new_n290), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n296), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n119), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n280), .b(new_n294), .c(new_n281), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .carry(new_n300));
  oaoi03aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n301));
  tech160nm_fixorc02aa1n03p5x5 g206(.a(\a[30] ), .b(\b[29] ), .out0(new_n302));
  inv000aa1d42x5               g207(.a(new_n302), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n301), .c(new_n277), .d(new_n299), .o1(new_n304));
  aoai13aa1n02x5               g209(.a(new_n299), .b(new_n286), .c(new_n210), .d(new_n275), .o1(new_n305));
  nona22aa1n02x4               g210(.a(new_n305), .b(new_n301), .c(new_n303), .out0(new_n306));
  nanp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  nanp02aa1n02x5               g212(.a(new_n301), .b(new_n302), .o1(new_n308));
  oai012aa1n02x5               g213(.a(new_n308), .b(\b[29] ), .c(\a[30] ), .o1(new_n309));
  nano23aa1n02x4               g214(.a(new_n294), .b(new_n281), .c(new_n302), .d(new_n280), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n286), .c(new_n210), .d(new_n275), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nona22aa1n02x4               g217(.a(new_n311), .b(new_n312), .c(new_n309), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n312), .b(new_n309), .c(new_n277), .d(new_n310), .o1(new_n314));
  nanp02aa1n03x5               g219(.a(new_n314), .b(new_n313), .o1(\s[31] ));
  xnbna2aa1n03x5               g220(.a(new_n120), .b(new_n121), .c(new_n115), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g224(.a(\a[5] ), .b(\b[4] ), .c(new_n161), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g226(.a(new_n104), .b(new_n161), .c(new_n126), .out0(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g228(.a(new_n107), .b(new_n322), .c(new_n102), .out0(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(new_n105), .out0(\s[8] ));
  xobna2aa1n03x5               g230(.a(new_n149), .b(new_n127), .c(new_n110), .out0(\s[9] ));
endmodule


