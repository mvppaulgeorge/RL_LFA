// Benchmark "adder" written by ABC on Wed Jul 10 17:17:53 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n316, new_n318;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n02x4               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  norp03aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nona23aa1n02x4               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  nanp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  oai012aa1n02x5               g019(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n108), .b(new_n109), .c(new_n107), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n116), .b(new_n111), .c(new_n115), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n101), .b(new_n100), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[6] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[5] ), .clkout(new_n120));
  norp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  oai122aa1n02x7               g027(.a(new_n118), .b(new_n103), .c(new_n122), .d(\b[7] ), .e(\a[8] ), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(new_n123), .b(new_n98), .c(new_n117), .d(new_n106), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n97), .b(new_n124), .out0(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  and002aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o(new_n127));
  norp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  160nm_ficinv00aa1n08x5       g035(.clk(new_n130), .clkout(new_n131));
  160nm_ficinv00aa1n08x5       g036(.clk(\a[10] ), .clkout(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(\b[9] ), .clkout(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n124), .c(\b[8] ), .d(\a[9] ), .o1(new_n135));
  nona22aa1n02x4               g040(.a(new_n135), .b(new_n131), .c(new_n127), .out0(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n130), .b(new_n135), .c(new_n132), .d(new_n133), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n128), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[12] ));
  xorc02aa1n02x5               g048(.a(\a[10] ), .b(\b[9] ), .out0(new_n144));
  nona23aa1n02x4               g049(.a(new_n141), .b(new_n129), .c(new_n128), .d(new_n140), .out0(new_n145));
  norb02aa1n02x5               g050(.a(new_n97), .b(new_n98), .out0(new_n146));
  nano22aa1n02x4               g051(.a(new_n145), .b(new_n144), .c(new_n146), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n123), .c(new_n106), .d(new_n117), .o1(new_n148));
  oaoi03aa1n02x5               g053(.a(new_n132), .b(new_n133), .c(new_n98), .o1(new_n149));
  oaoi03aa1n02x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n139), .o1(new_n150));
  oabi12aa1n02x5               g055(.a(new_n150), .b(new_n145), .c(new_n149), .out0(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  nanp02aa1n02x5               g057(.a(new_n148), .b(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nona23aa1n02x4               g065(.a(new_n160), .b(new_n156), .c(new_n155), .d(new_n159), .out0(new_n161));
  aoi012aa1n02x5               g066(.a(new_n159), .b(new_n155), .c(new_n160), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n148), .d(new_n152), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n128), .b(new_n140), .c(new_n141), .d(new_n129), .out0(new_n174));
  nanb02aa1n02x5               g079(.a(new_n165), .b(new_n166), .out0(new_n175));
  nano23aa1n02x4               g080(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n176), .b(new_n169), .c(new_n175), .out0(new_n177));
  nano32aa1n02x4               g082(.a(new_n177), .b(new_n174), .c(new_n146), .d(new_n144), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n123), .c(new_n106), .d(new_n117), .o1(new_n179));
  norp03aa1n02x5               g084(.a(new_n161), .b(new_n169), .c(new_n175), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n181));
  nona23aa1n02x4               g086(.a(new_n168), .b(new_n166), .c(new_n165), .d(new_n167), .out0(new_n182));
  oai022aa1n02x5               g087(.a(new_n182), .b(new_n162), .c(\b[15] ), .d(\a[16] ), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n181), .c(new_n151), .d(new_n180), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n179), .b(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d04x5               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  norp02aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  aoi013aa1n02x4               g100(.a(new_n194), .b(new_n195), .c(new_n188), .d(new_n189), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n179), .d(new_n184), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aoi112aa1n02x5               g109(.a(new_n200), .b(new_n204), .c(new_n197), .d(new_n201), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n204), .b(new_n200), .c(new_n197), .d(new_n201), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g112(.a(new_n200), .b(new_n202), .c(new_n203), .d(new_n201), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n192), .b(new_n208), .o1(new_n209));
  nona23aa1n02x4               g114(.a(new_n203), .b(new_n201), .c(new_n200), .d(new_n202), .out0(new_n210));
  oai012aa1n02x5               g115(.a(new_n203), .b(new_n202), .c(new_n200), .o1(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n210), .c(new_n196), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n209), .c(new_n179), .d(new_n184), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[21] ), .clkout(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[22] ), .clkout(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n192), .c(new_n208), .o1(new_n225));
  nona22aa1n02x4               g130(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(new_n226));
  oaib12aa1n02x5               g131(.a(new_n226), .b(\b[17] ), .c(new_n187), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n211), .clkout(new_n228));
  aoai13aa1n02x5               g133(.a(new_n224), .b(new_n228), .c(new_n208), .d(new_n227), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\b[21] ), .clkout(new_n230));
  oao003aa1n02x5               g135(.a(new_n223), .b(new_n230), .c(new_n216), .carry(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  nanp02aa1n02x5               g137(.a(new_n229), .b(new_n232), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n225), .c(new_n179), .d(new_n184), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoi112aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoai13aa1n02x5               g145(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(\s[24] ));
  and002aa1n02x5               g147(.a(new_n239), .b(new_n238), .o(new_n243));
  nanb03aa1n02x5               g148(.a(new_n209), .b(new_n243), .c(new_n224), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n243), .clkout(new_n245));
  nanp02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  oai022aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n245), .c(new_n229), .d(new_n232), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n244), .c(new_n179), .d(new_n184), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n253), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n257), .b(new_n256), .out0(\s[26] ));
  norp02aa1n02x5               g163(.a(new_n103), .b(new_n122), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n259), .b(new_n99), .c(new_n100), .d(new_n101), .o1(new_n260));
  aob012aa1n02x5               g165(.a(new_n260), .b(new_n117), .c(new_n106), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n151), .b(new_n180), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n183), .c(new_n181), .out0(new_n263));
  and002aa1n02x5               g168(.a(new_n255), .b(new_n254), .o(new_n264));
  nano22aa1n02x4               g169(.a(new_n225), .b(new_n243), .c(new_n264), .out0(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n178), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aob012aa1n02x5               g172(.a(new_n267), .b(\b[25] ), .c(\a[26] ), .out0(new_n268));
  aobi12aa1n02x5               g173(.a(new_n268), .b(new_n249), .c(new_n264), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n266), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  aobi12aa1n02x5               g178(.a(new_n270), .b(new_n269), .c(new_n266), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n274), .b(new_n273), .c(new_n275), .out0(new_n276));
  aobi12aa1n02x5               g181(.a(new_n265), .b(new_n179), .c(new_n184), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n243), .b(new_n231), .c(new_n212), .d(new_n224), .o1(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n264), .clkout(new_n279));
  aoai13aa1n02x5               g184(.a(new_n268), .b(new_n279), .c(new_n278), .d(new_n248), .o1(new_n280));
  oai012aa1n02x5               g185(.a(new_n270), .b(new_n280), .c(new_n277), .o1(new_n281));
  aoi012aa1n02x5               g186(.a(new_n275), .b(new_n281), .c(new_n273), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n282), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n270), .b(new_n275), .out0(new_n284));
  aobi12aa1n02x5               g189(.a(new_n284), .b(new_n269), .c(new_n266), .out0(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n285), .b(new_n286), .c(new_n287), .out0(new_n288));
  oai012aa1n02x5               g193(.a(new_n284), .b(new_n280), .c(new_n277), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n287), .b(new_n289), .c(new_n286), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n270), .b(new_n287), .c(new_n275), .out0(new_n293));
  aobi12aa1n02x5               g198(.a(new_n293), .b(new_n269), .c(new_n266), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n293), .b(new_n280), .c(new_n277), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n293), .b(new_n296), .out0(new_n301));
  aobi12aa1n02x5               g206(.a(new_n301), .b(new_n269), .c(new_n266), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n301), .b(new_n280), .c(new_n277), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g217(.a(new_n121), .b(new_n117), .c(new_n105), .out0(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  norp02aa1n02x5               g219(.a(new_n105), .b(new_n104), .o1(new_n315));
  aob012aa1n02x5               g220(.a(new_n122), .b(new_n117), .c(new_n315), .out0(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g222(.a(new_n101), .b(new_n316), .c(new_n102), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n261), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


