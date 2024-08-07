// Benchmark "adder" written by ABC on Thu Jul 18 10:07:04 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n319, new_n320,
    new_n321, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nand42aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nano23aa1n03x5               g007(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[6] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[5] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  oao003aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .carry(new_n107));
  inv000aa1d42x5               g012(.a(new_n101), .o1(new_n108));
  oaoi03aa1n02x5               g013(.a(\a[8] ), .b(\b[7] ), .c(new_n108), .o1(new_n109));
  tech160nm_fiaoi012aa1n02p5x5 g014(.a(new_n109), .b(new_n103), .c(new_n107), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aoi012aa1d24x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  inv000aa1d42x5               g019(.a(new_n114), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  norp02aa1n04x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  tech160nm_fiaoi012aa1n03p5x5 g026(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n122));
  xorc02aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .out0(new_n123));
  tech160nm_fixorc02aa1n02p5x5 g028(.a(\a[5] ), .b(\b[4] ), .out0(new_n124));
  nanp03aa1n02x5               g029(.a(new_n103), .b(new_n123), .c(new_n124), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n110), .b(new_n125), .c(new_n121), .d(new_n122), .o1(new_n126));
  tech160nm_fiaoi012aa1n05x5   g031(.a(new_n97), .b(new_n126), .c(new_n98), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oaoi03aa1n03x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n06x4               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  norp02aa1n04x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  tech160nm_finand02aa1n05x5   g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norb02aa1n09x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  aoai13aa1n03x5               g042(.a(new_n137), .b(new_n131), .c(new_n129), .d(new_n133), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n129), .b(new_n133), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n137), .c(new_n131), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n140), .b(new_n138), .o1(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[9] ), .b(\a[10] ), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n143));
  oai112aa1n04x5               g048(.a(new_n136), .b(new_n133), .c(new_n143), .d(new_n142), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n134), .b(new_n131), .c(new_n135), .o1(new_n145));
  and002aa1n02x5               g050(.a(new_n144), .b(new_n145), .o(new_n146));
  nona23aa1n03x5               g051(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n147));
  oaoi03aa1n02x5               g052(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n148));
  oabi12aa1n02x5               g053(.a(new_n109), .b(new_n147), .c(new_n148), .out0(new_n149));
  nona23aa1n02x4               g054(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n150));
  oai012aa1n06x5               g055(.a(new_n122), .b(new_n150), .c(new_n114), .o1(new_n151));
  nano22aa1n02x5               g056(.a(new_n147), .b(new_n123), .c(new_n124), .out0(new_n152));
  xnrc02aa1n02x5               g057(.a(\b[9] ), .b(\a[10] ), .out0(new_n153));
  nanb02aa1n02x5               g058(.a(new_n97), .b(new_n98), .out0(new_n154));
  nano23aa1n02x4               g059(.a(new_n154), .b(new_n153), .c(new_n133), .d(new_n136), .out0(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n149), .c(new_n152), .d(new_n151), .o1(new_n156));
  tech160nm_fixorc02aa1n04x5   g061(.a(\a[13] ), .b(\b[12] ), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n156), .c(new_n146), .out0(\s[13] ));
  inv000aa1d42x5               g063(.a(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n156), .b(new_n146), .o1(new_n160));
  norp02aa1n04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n157), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(new_n159), .out0(\s[14] ));
  xorc02aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .out0(new_n164));
  and002aa1n02x5               g069(.a(new_n164), .b(new_n157), .o(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  inv000aa1d42x5               g071(.a(\b[13] ), .o1(new_n167));
  oao003aa1n09x5               g072(.a(new_n159), .b(new_n167), .c(new_n161), .carry(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n04x5               g074(.a(new_n169), .b(new_n166), .c(new_n156), .d(new_n146), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n06x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n03x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n177));
  aoi112aa1n03x5               g082(.a(new_n176), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(\s[16] ));
  nano23aa1n03x5               g084(.a(new_n172), .b(new_n174), .c(new_n175), .d(new_n173), .out0(new_n180));
  tech160nm_fiao0012aa1n02p5x5 g085(.a(new_n174), .b(new_n172), .c(new_n175), .o(new_n181));
  tech160nm_fiaoi012aa1n02p5x5 g086(.a(new_n181), .b(new_n180), .c(new_n168), .o1(new_n182));
  nand23aa1n03x5               g087(.a(new_n180), .b(new_n157), .c(new_n164), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n182), .b(new_n183), .c(new_n145), .d(new_n144), .o1(new_n184));
  inv040aa1n04x5               g089(.a(new_n184), .o1(new_n185));
  nano23aa1n02x4               g090(.a(new_n131), .b(new_n134), .c(new_n135), .d(new_n132), .out0(new_n186));
  norp02aa1n02x5               g091(.a(new_n153), .b(new_n154), .o1(new_n187));
  nano22aa1n03x7               g092(.a(new_n183), .b(new_n187), .c(new_n186), .out0(new_n188));
  aoai13aa1n09x5               g093(.a(new_n188), .b(new_n149), .c(new_n152), .d(new_n151), .o1(new_n189));
  nanp02aa1n06x5               g094(.a(new_n189), .b(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g096(.a(\a[18] ), .o1(new_n192));
  inv040aa1d30x5               g097(.a(\a[17] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[16] ), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n190), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(new_n192), .out0(\s[18] ));
  xroi22aa1d06x4               g101(.a(new_n193), .b(\b[16] ), .c(new_n192), .d(\b[17] ), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  nor042aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  aoi112aa1n09x5               g104(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n200));
  nor022aa1n04x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n198), .c(new_n189), .d(new_n185), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand02aa1d04x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nor042aa1n06x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand02aa1n20x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n12x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n205), .c(new_n202), .d(new_n206), .o1(new_n211));
  norb02aa1n03x5               g116(.a(new_n206), .b(new_n205), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n202), .b(new_n212), .o1(new_n213));
  nona22aa1n02x4               g118(.a(new_n213), .b(new_n210), .c(new_n205), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n214), .b(new_n211), .o1(\s[20] ));
  nona23aa1n09x5               g120(.a(new_n208), .b(new_n206), .c(new_n205), .d(new_n207), .out0(new_n216));
  tech160nm_fiaoi012aa1n04x5   g121(.a(new_n207), .b(new_n205), .c(new_n208), .o1(new_n217));
  oai012aa1n12x5               g122(.a(new_n217), .b(new_n216), .c(new_n201), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  nanb02aa1n06x5               g124(.a(new_n216), .b(new_n197), .out0(new_n220));
  aoai13aa1n04x5               g125(.a(new_n219), .b(new_n220), .c(new_n189), .d(new_n185), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n04x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n12x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  tech160nm_fixnrc02aa1n05x5   g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(new_n221), .b(new_n224), .o1(new_n227));
  nona22aa1n02x4               g132(.a(new_n227), .b(new_n225), .c(new_n223), .out0(new_n228));
  nanp02aa1n02x5               g133(.a(new_n228), .b(new_n226), .o1(\s[22] ));
  oai112aa1n06x5               g134(.a(new_n212), .b(new_n209), .c(new_n200), .d(new_n199), .o1(new_n230));
  nanb02aa1n02x5               g135(.a(new_n225), .b(new_n224), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\a[22] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\b[21] ), .o1(new_n233));
  oaoi03aa1n12x5               g138(.a(new_n232), .b(new_n233), .c(new_n223), .o1(new_n234));
  aoai13aa1n12x5               g139(.a(new_n234), .b(new_n231), .c(new_n230), .d(new_n217), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  nona23aa1n08x5               g141(.a(new_n197), .b(new_n224), .c(new_n225), .d(new_n216), .out0(new_n237));
  aoai13aa1n04x5               g142(.a(new_n236), .b(new_n237), .c(new_n189), .d(new_n185), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n12x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(new_n238), .b(new_n241), .o1(new_n245));
  nona22aa1n02x4               g150(.a(new_n245), .b(new_n243), .c(new_n240), .out0(new_n246));
  nanp02aa1n03x5               g151(.a(new_n246), .b(new_n244), .o1(\s[24] ));
  norb02aa1n02x5               g152(.a(new_n224), .b(new_n225), .out0(new_n248));
  and002aa1n02x5               g153(.a(new_n242), .b(new_n241), .o(new_n249));
  nano22aa1n03x7               g154(.a(new_n220), .b(new_n249), .c(new_n248), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n184), .c(new_n126), .d(new_n188), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n234), .o1(new_n252));
  aoai13aa1n04x5               g157(.a(new_n249), .b(new_n252), .c(new_n218), .d(new_n248), .o1(new_n253));
  inv000aa1d42x5               g158(.a(\a[24] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(\b[23] ), .o1(new_n255));
  oaoi03aa1n12x5               g160(.a(new_n254), .b(new_n255), .c(new_n240), .o1(new_n256));
  nanp02aa1n03x5               g161(.a(new_n253), .b(new_n256), .o1(new_n257));
  nanb02aa1n03x5               g162(.a(new_n257), .b(new_n251), .out0(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xorc02aa1n12x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n261), .b(new_n257), .c(new_n190), .d(new_n250), .o1(new_n265));
  nona22aa1n02x5               g170(.a(new_n265), .b(new_n263), .c(new_n260), .out0(new_n266));
  nanp02aa1n03x5               g171(.a(new_n264), .b(new_n266), .o1(\s[26] ));
  inv000aa1d42x5               g172(.a(new_n256), .o1(new_n268));
  and002aa1n06x5               g173(.a(new_n262), .b(new_n261), .o(new_n269));
  aoai13aa1n04x5               g174(.a(new_n269), .b(new_n268), .c(new_n235), .d(new_n249), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n271));
  oab012aa1n02x4               g176(.a(new_n271), .b(\a[26] ), .c(\b[25] ), .out0(new_n272));
  nano22aa1n03x7               g177(.a(new_n237), .b(new_n249), .c(new_n269), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n184), .c(new_n126), .d(new_n188), .o1(new_n274));
  nand43aa1n04x5               g179(.a(new_n274), .b(new_n270), .c(new_n272), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n277), .c(new_n275), .d(new_n278), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n269), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n272), .b(new_n281), .c(new_n253), .d(new_n256), .o1(new_n282));
  aobi12aa1n06x5               g187(.a(new_n273), .b(new_n189), .c(new_n185), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n278), .b(new_n282), .c(new_n283), .o1(new_n284));
  nona22aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n277), .out0(new_n285));
  nanp02aa1n03x5               g190(.a(new_n280), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n278), .b(new_n279), .out0(new_n287));
  oaih12aa1n02x5               g192(.a(new_n287), .b(new_n282), .c(new_n283), .o1(new_n288));
  aob012aa1n02x5               g193(.a(new_n277), .b(\b[27] ), .c(\a[28] ), .out0(new_n289));
  oa0012aa1n12x5               g194(.a(new_n289), .b(\b[27] ), .c(\a[28] ), .o(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  nona22aa1n02x5               g197(.a(new_n288), .b(new_n291), .c(new_n292), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n292), .b(new_n291), .c(new_n275), .d(new_n287), .o1(new_n294));
  nanp02aa1n03x5               g199(.a(new_n294), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n278), .b(new_n292), .c(new_n279), .out0(new_n297));
  oaoi03aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .o1(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n275), .d(new_n297), .o1(new_n300));
  oaih12aa1n02x5               g205(.a(new_n297), .b(new_n282), .c(new_n283), .o1(new_n301));
  nona22aa1n02x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  nanb02aa1n02x5               g208(.a(new_n299), .b(new_n298), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(\b[29] ), .c(\a[30] ), .o1(new_n305));
  norb02aa1n02x5               g210(.a(new_n297), .b(new_n299), .out0(new_n306));
  oaih12aa1n02x5               g211(.a(new_n306), .b(new_n282), .c(new_n283), .o1(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nona22aa1n02x5               g213(.a(new_n307), .b(new_n308), .c(new_n305), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n305), .c(new_n275), .d(new_n306), .o1(new_n310));
  nanp02aa1n03x5               g215(.a(new_n310), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n151), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n03x5               g220(.a(new_n106), .b(new_n151), .c(new_n124), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(new_n104), .out0(\s[6] ));
  and002aa1n02x5               g222(.a(\b[5] ), .b(\a[6] ), .o(new_n318));
  nanp02aa1n02x5               g223(.a(new_n316), .b(new_n123), .o1(new_n319));
  nona23aa1n03x5               g224(.a(new_n319), .b(new_n102), .c(new_n101), .d(new_n318), .out0(new_n320));
  aboi22aa1n03x5               g225(.a(new_n318), .b(new_n319), .c(new_n108), .d(new_n102), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n320), .b(new_n321), .out0(\s[7] ));
  norb02aa1n02x5               g227(.a(new_n100), .b(new_n99), .out0(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n323), .b(new_n320), .c(new_n108), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


