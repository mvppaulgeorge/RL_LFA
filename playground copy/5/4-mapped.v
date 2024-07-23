// Benchmark "adder" written by ABC on Thu Jul 11 11:12:32 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n288, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n308, new_n310,
    new_n312, new_n314, new_n316, new_n318;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(new_n98), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n02x4               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  160nm_fiao0012aa1n02p5x5     g012(.a(new_n105), .b(new_n107), .c(new_n106), .o(new_n108));
  160nm_fiao0012aa1n02p5x5     g013(.a(new_n100), .b(new_n102), .c(new_n101), .o(new_n109));
  aoi012aa1n02x5               g014(.a(new_n109), .b(new_n104), .c(new_n108), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  aoi012aa1n02x5               g018(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n114));
  norb02aa1n02x5               g019(.a(new_n112), .b(new_n111), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  norb02aa1n02x5               g021(.a(new_n116), .b(new_n113), .out0(new_n117));
  norp02aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  aoi022aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n119));
  oai112aa1n02x5               g024(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n119), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nano23aa1n02x4               g026(.a(new_n105), .b(new_n107), .c(new_n121), .d(new_n106), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n104), .o1(new_n123));
  aoai13aa1n02x5               g028(.a(new_n110), .b(new_n123), .c(new_n114), .d(new_n120), .o1(new_n124));
  aob012aa1n02x5               g029(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n97), .b(new_n125), .c(new_n99), .out0(\s[10] ));
  orn002aa1n02x5               g031(.a(\a[10] ), .b(\b[9] ), .o(new_n127));
  and002aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o(new_n128));
  aoi013aa1n02x4               g033(.a(new_n128), .b(new_n125), .c(new_n99), .d(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  norp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n131), .c(new_n129), .d(new_n133), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n129), .b(new_n133), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n138), .b(new_n136), .c(new_n131), .out0(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n137), .o1(\s[12] ));
  nano23aa1n02x4               g045(.a(new_n131), .b(new_n134), .c(new_n135), .d(new_n132), .out0(new_n141));
  oaoi03aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n134), .b(new_n131), .c(new_n135), .o1(new_n143));
  aobi12aa1n02x5               g048(.a(new_n143), .b(new_n141), .c(new_n142), .out0(new_n144));
  xorc02aa1n02x5               g049(.a(\a[9] ), .b(\b[8] ), .out0(new_n145));
  nanp03aa1n02x5               g050(.a(new_n141), .b(new_n97), .c(new_n145), .o1(new_n146));
  nanb02aa1n02x5               g051(.a(new_n146), .b(new_n124), .out0(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n144), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n150), .b(new_n148), .c(new_n151), .o1(new_n152));
  xnrb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nano23aa1n02x4               g060(.a(new_n150), .b(new_n154), .c(new_n155), .d(new_n151), .out0(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  oa0012aa1n02x5               g062(.a(new_n155), .b(new_n154), .c(new_n150), .o(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n157), .c(new_n147), .d(new_n144), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  norp02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n162), .c(new_n160), .d(new_n164), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n160), .b(new_n164), .o1(new_n169));
  nona22aa1n02x4               g074(.a(new_n169), .b(new_n167), .c(new_n162), .out0(new_n170));
  nanp02aa1n02x5               g075(.a(new_n170), .b(new_n168), .o1(\s[16] ));
  nano23aa1n02x4               g076(.a(new_n162), .b(new_n165), .c(new_n166), .d(new_n163), .out0(new_n172));
  160nm_fiao0012aa1n02p5x5     g077(.a(new_n165), .b(new_n162), .c(new_n166), .o(new_n173));
  aoi012aa1n02x5               g078(.a(new_n173), .b(new_n172), .c(new_n158), .o1(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n174), .clkout(new_n175));
  nanp02aa1n02x5               g080(.a(new_n172), .b(new_n156), .o1(new_n176));
  oab012aa1n02x4               g081(.a(new_n175), .b(new_n144), .c(new_n176), .out0(new_n177));
  nano32aa1n02x4               g082(.a(new_n176), .b(new_n141), .c(new_n145), .d(new_n97), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n124), .b(new_n178), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n177), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[18] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[17] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\b[16] ), .clkout(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  oai012aa1n02x5               g091(.a(new_n174), .b(new_n144), .c(new_n176), .o1(new_n187));
  xroi22aa1d04x5               g092(.a(new_n183), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n124), .d(new_n178), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n191));
  norp02aa1n02x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n189), .c(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g102(.a(new_n189), .b(new_n192), .o1(new_n198));
  norp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n201), .clkout(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n193), .c(new_n198), .d(new_n194), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(new_n198), .b(new_n195), .o1(new_n204));
  nona22aa1n02x4               g109(.a(new_n204), .b(new_n202), .c(new_n193), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n205), .b(new_n203), .o1(\s[20] ));
  nona23aa1n02x4               g111(.a(new_n200), .b(new_n194), .c(new_n193), .d(new_n199), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n199), .b(new_n193), .c(new_n200), .o1(new_n208));
  oai012aa1n02x5               g113(.a(new_n208), .b(new_n207), .c(new_n192), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  nanb02aa1n02x5               g115(.a(new_n207), .b(new_n188), .out0(new_n211));
  aoai13aa1n02x5               g116(.a(new_n210), .b(new_n211), .c(new_n179), .d(new_n177), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xnrc02aa1n02x5               g120(.a(\b[21] ), .b(\a[22] ), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(new_n212), .b(new_n215), .o1(new_n218));
  nona22aa1n02x4               g123(.a(new_n218), .b(new_n216), .c(new_n214), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n219), .b(new_n217), .o1(\s[22] ));
  oai112aa1n02x5               g125(.a(new_n195), .b(new_n201), .c(new_n191), .d(new_n190), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(new_n216), .b(new_n215), .out0(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[22] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\b[21] ), .clkout(new_n224));
  oaoi03aa1n02x5               g129(.a(new_n223), .b(new_n224), .c(new_n214), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n222), .c(new_n221), .d(new_n208), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  nona23aa1n02x4               g132(.a(new_n188), .b(new_n215), .c(new_n216), .d(new_n207), .out0(new_n228));
  aoai13aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n179), .d(new_n177), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n229), .b(new_n232), .o1(new_n236));
  nona22aa1n02x4               g141(.a(new_n236), .b(new_n234), .c(new_n231), .out0(new_n237));
  nanp02aa1n02x5               g142(.a(new_n237), .b(new_n235), .o1(\s[24] ));
  norb02aa1n02x5               g143(.a(new_n215), .b(new_n216), .out0(new_n239));
  and002aa1n02x5               g144(.a(new_n233), .b(new_n232), .o(new_n240));
  nano22aa1n02x4               g145(.a(new_n211), .b(new_n240), .c(new_n239), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n187), .c(new_n124), .d(new_n178), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n225), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n240), .b(new_n243), .c(new_n209), .d(new_n239), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(\a[24] ), .clkout(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(\b[23] ), .clkout(new_n246));
  oaoi03aa1n02x5               g151(.a(new_n245), .b(new_n246), .c(new_n231), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n244), .b(new_n247), .o1(new_n248));
  nanb02aa1n02x5               g153(.a(new_n248), .b(new_n242), .out0(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n253), .clkout(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n251), .c(new_n249), .d(new_n252), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n252), .b(new_n248), .c(new_n180), .d(new_n241), .o1(new_n256));
  nona22aa1n02x4               g161(.a(new_n256), .b(new_n254), .c(new_n251), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n255), .b(new_n257), .o1(\s[26] ));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n247), .clkout(new_n259));
  and002aa1n02x5               g164(.a(new_n253), .b(new_n252), .o(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n259), .c(new_n226), .d(new_n240), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n262));
  oab012aa1n02x4               g167(.a(new_n262), .b(\a[26] ), .c(\b[25] ), .out0(new_n263));
  nano22aa1n02x4               g168(.a(new_n228), .b(new_n240), .c(new_n260), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n187), .c(new_n124), .d(new_n178), .o1(new_n265));
  nanp03aa1n02x5               g170(.a(new_n265), .b(new_n261), .c(new_n263), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n260), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n263), .b(new_n272), .c(new_n244), .d(new_n247), .o1(new_n273));
  aobi12aa1n02x5               g178(.a(new_n264), .b(new_n179), .c(new_n177), .out0(new_n274));
  oai012aa1n02x5               g179(.a(new_n269), .b(new_n273), .c(new_n274), .o1(new_n275));
  nona22aa1n02x4               g180(.a(new_n275), .b(new_n270), .c(new_n268), .out0(new_n276));
  nanp02aa1n02x5               g181(.a(new_n276), .b(new_n271), .o1(\s[28] ));
  norb02aa1n02x5               g182(.a(new_n269), .b(new_n270), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n278), .b(new_n273), .c(new_n274), .o1(new_n279));
  aob012aa1n02x5               g184(.a(new_n268), .b(\b[27] ), .c(\a[28] ), .out0(new_n280));
  oai012aa1n02x5               g185(.a(new_n280), .b(\b[27] ), .c(\a[28] ), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .o1(new_n282));
  and002aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .o(new_n283));
  nona32aa1n02x4               g188(.a(new_n279), .b(new_n283), .c(new_n282), .d(new_n281), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n281), .c(new_n266), .d(new_n278), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(new_n286), .b(new_n284), .o1(\s[29] ));
  nanp02aa1n02x5               g192(.a(\b[0] ), .b(\a[1] ), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n269), .b(new_n285), .c(new_n270), .out0(new_n290));
  160nm_ficinv00aa1n08x5       g195(.clk(new_n283), .clkout(new_n291));
  160nm_fiao0012aa1n02p5x5     g196(.a(new_n282), .b(new_n281), .c(new_n291), .o(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n292), .c(new_n266), .d(new_n290), .o1(new_n294));
  oai012aa1n02x5               g199(.a(new_n290), .b(new_n273), .c(new_n274), .o1(new_n295));
  norp02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .o1(new_n296));
  and002aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .o(new_n297));
  nona32aa1n02x4               g202(.a(new_n295), .b(new_n297), .c(new_n296), .d(new_n292), .out0(new_n298));
  nanp02aa1n02x5               g203(.a(new_n294), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n292), .b(new_n293), .out0(new_n300));
  norb02aa1n02x5               g205(.a(new_n290), .b(new_n293), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n301), .b(new_n273), .c(new_n274), .o1(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nona32aa1n02x4               g208(.a(new_n302), .b(new_n303), .c(new_n300), .d(new_n296), .out0(new_n304));
  oabi12aa1n02x5               g209(.a(new_n300), .b(\a[30] ), .c(\b[29] ), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n266), .d(new_n301), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  norp02aa1n02x5               g212(.a(new_n119), .b(new_n118), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(new_n308), .b(new_n117), .out0(\s[3] ));
  oaoi13aa1n02x5               g214(.a(new_n113), .b(new_n116), .c(new_n119), .d(new_n118), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(new_n310), .b(new_n115), .out0(\s[4] ));
  nanp02aa1n02x5               g216(.a(new_n120), .b(new_n114), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g218(.a(new_n107), .b(new_n312), .c(new_n121), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g220(.a(new_n108), .b(new_n312), .c(new_n122), .o(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g222(.a(new_n102), .b(new_n316), .c(new_n103), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


